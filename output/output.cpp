/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * output.cpp - video stream output base class
 */

#include <chrono>
#include <cinttypes>
#include <filesystem>
#include <stdexcept>

#include <libcamera/control_ids.h>

#include "circular_output.hpp"
#include "file_output.hpp"
#include "net_output.hpp"
#include "output.hpp"

namespace fs = std::filesystem;

Output::Output(VideoOptions const *options)
	: options_(options), fp_timestamps_(nullptr), state_(WAITING_KEYFRAME), time_offset_(0), last_timestamp_(0),
	  buf_metadata_(std::cout.rdbuf()), of_metadata_(), last_metadata_flush_{}
{
	if (!options->Get().save_pts.empty())
	{
		fp_timestamps_ = fopen(options->Get().save_pts.c_str(), "w");
		if (!fp_timestamps_)
			throw std::runtime_error("Failed to open timestamp file " + options->Get().save_pts);
		fprintf(fp_timestamps_, "# timecode format v2\n");
	}
	if (options->Get().MetadataEnabled())
	{
		const std::string &fmt = options_->Get().metadata_format;
		if (fmt == "jsonl")
		{
			fs::create_directories(options_->Get().metadata_dir);
		}
		else
		{
			const std::string &filename = options_->Get().metadata;
			if (filename.compare("-"))
			{
				of_metadata_.open(filename, std::ios::out);
				buf_metadata_ = of_metadata_.rdbuf();
				start_metadata_output(buf_metadata_, fmt);
			}
		}
	}

	enable_ = !options->Get().pause;
}

Output::~Output()
{
	if (fp_timestamps_)
		fclose(fp_timestamps_);
	if (options_->Get().MetadataEnabled())
	{
		if (options_->Get().metadata_format != "jsonl")
			stop_metadata_output(buf_metadata_, options_->Get().metadata_format);
		else if (of_jsonl_.is_open())
			of_jsonl_.close();
	}
}

void Output::Signal()
{
	enable_ = !enable_;
}

void Output::OutputReady(void *mem, size_t size, int64_t timestamp_us, bool keyframe)
{
	// When output is enabled, we may have to wait for the next keyframe.
	uint32_t flags = keyframe ? FLAG_KEYFRAME : FLAG_NONE;
	if (!enable_)
		state_ = DISABLED;
	else if (state_ == DISABLED)
		state_ = WAITING_KEYFRAME;
	if (state_ == WAITING_KEYFRAME && keyframe)
		state_ = RUNNING, flags |= FLAG_RESTART;
	if (state_ != RUNNING)
		return;

	// Frig the timestamps to be continuous after a pause.
	if (flags & FLAG_RESTART)
		time_offset_ = timestamp_us - last_timestamp_;
	last_timestamp_ = timestamp_us - time_offset_;

	outputBuffer(mem, size, last_timestamp_, flags);

	// Save timestamps to a file, if that was requested.
	if (fp_timestamps_)
	{
		timestampReady(last_timestamp_);
	}

	if (!options_->Get().MetadataEnabled() || metadata_queue_.empty())
		return;

	const std::string &fmt = options_->Get().metadata_format;
	libcamera::ControlList metadata = metadata_queue_.front();
	metadata_queue_.pop();

	if (fmt == "jsonl")
	{
		// Bucket epoch (seconds) from FrameWallClock (nanoseconds); bucket aligns to rotate interval.
		auto fwc = metadata.get(libcamera::controls::FrameWallClock);
		int64_t fwc_ns = fwc ? *fwc : 0;
		int64_t sec = fwc_ns / 1000000000;
		unsigned int rotate_secs = options_->Get().metadata_rotate_secs;
		int64_t bucket_epoch_sec = (sec / static_cast<int64_t>(rotate_secs)) * rotate_secs;

		// Ensure the right bucket file is open; close previous and cleanup old buckets.
		if (of_jsonl_.is_open() && current_jsonl_bucket_epoch_ != bucket_epoch_sec)
		{
			of_jsonl_.close();
			current_jsonl_bucket_epoch_ = -1;
		}
		if (!of_jsonl_.is_open())
		{
			fs::path dir(options_->Get().metadata_dir);
			fs::path path = dir / (std::to_string(bucket_epoch_sec) + ".jsonl");
			of_jsonl_.open(path, std::ios::out | std::ios::app);
			if (!of_jsonl_)
				throw std::runtime_error("Failed to open metadata JSONL file " + path.string());
			current_jsonl_bucket_epoch_ = bucket_epoch_sec;

			// Remove bucket files older than max_mins, at most once every 2 * rotate_secs.
			unsigned int max_mins = options_->Get().metadata_max_mins;
			int64_t prune_interval_sec = static_cast<int64_t>(2 * rotate_secs);
			bool should_prune = max_mins > 0 &&
			                    (last_jsonl_prune_bucket_epoch_ < 0 ||
			                     (bucket_epoch_sec - last_jsonl_prune_bucket_epoch_) >= prune_interval_sec);
			if (should_prune)
			{
				last_jsonl_prune_bucket_epoch_ = bucket_epoch_sec;
				int64_t cutoff_epoch = bucket_epoch_sec - static_cast<int64_t>(max_mins) * 60;
				try
				{
					for (const auto &entry : fs::directory_iterator(dir))
					{
						if (!entry.is_regular_file())
							continue;
						std::string name = entry.path().filename().string();
						if (name.size() < 7 || name.compare(name.size() - 6, 6, ".jsonl") != 0)
							continue;
						std::string base = name.substr(0, name.size() - 6);
						int64_t epoch = 0;
						try
						{
							epoch = std::stoll(base);
						}
						catch (...)
						{
							continue;
						}
						if (epoch < cutoff_epoch)
							fs::remove(entry.path());
					}
				}
				catch (const fs::filesystem_error &)
				{
					// Ignore cleanup errors (e.g. permission, concurrent delete).
				}
			}
		}
		write_metadata_jsonl_line(of_jsonl_, metadata);
		if (options_->Get().flush)
		{
			auto interval_ms = options_->Get().metadata_flush_interval;
			auto now = std::chrono::steady_clock::now();
			if (interval_ms == 0 ||
			    now - last_metadata_flush_ >= std::chrono::milliseconds(interval_ms))
			{
				of_jsonl_.flush();
				last_metadata_flush_ = now;
			}
		}
	}
	else
	{
		write_metadata(buf_metadata_, fmt, metadata, !metadata_started_);
		metadata_started_ = true;
		if (options_->Get().flush && of_metadata_.is_open())
		{
			auto interval_ms = options_->Get().metadata_flush_interval;
			auto now = std::chrono::steady_clock::now();
			if (interval_ms == 0 ||
			    now - last_metadata_flush_ >= std::chrono::milliseconds(interval_ms))
			{
				of_metadata_.flush();
				last_metadata_flush_ = now;
			}
		}
	}
}

void Output::timestampReady(int64_t timestamp)
{
	fprintf(fp_timestamps_, "%" PRId64 ".%03" PRId64 "\n", timestamp / 1000, timestamp % 1000);
	if (options_->Get().flush)
		fflush(fp_timestamps_);
}

void Output::outputBuffer(void *mem, size_t size, int64_t timestamp_us, uint32_t flags)
{
	// Supply this so that a vanilla Output gives you an object that outputs no buffers.
}

Output *Output::Create(VideoOptions const *options)
{
	bool libav = options->Get().codec == "libav" ||
				 (options->Get().codec == "h264" && options->GetPlatform() != Platform::VC4);
	const std::string out_file = options->Get().output;

	if (!libav && (strncmp(out_file.c_str(), "udp://", 6) == 0 || strncmp(out_file.c_str(), "tcp://", 6) == 0))
		return new NetOutput(options);
	else if (options->Get().circular)
		return new CircularOutput(options);
	else if (!out_file.empty())
		return new FileOutput(options);
	else
		return new Output(options);
}

void Output::MetadataReady(libcamera::ControlList &metadata)
{
	if (!options_->Get().MetadataEnabled())
		return;

	metadata_queue_.push(metadata);
}

void start_metadata_output(std::streambuf *buf, std::string fmt)
{
	std::ostream out(buf);
	if (fmt == "json")
		out << "[" << std::endl;
}

void write_metadata(std::streambuf *buf, std::string fmt, libcamera::ControlList &metadata, bool first_write)
{
	std::ostream out(buf);
	const libcamera::ControlIdMap *id_map = metadata.idMap();
	if (fmt == "txt")
	{
		for (auto const &[id, val] : metadata)
			out << id_map->at(id)->name() << "=" << val.toString() << std::endl;
		out << std::endl;
	}
	else if (fmt == "jsonl")
	{
		write_metadata_jsonl_line(out, metadata);
	}
	else
	{
		if (!first_write)
			out << "," << std::endl;
		out << "{";
		bool first_done = false;
		for (auto const &[id, val] : metadata)
		{
			std::string arg_quote = (val.toString().find('/') != std::string::npos) ? "\"" : "";
			out << (first_done ? "," : "") << std::endl
				<< "    \"" << id_map->at(id)->name() << "\": " << arg_quote << val.toString() << arg_quote;
			first_done = true;
		}
		out << std::endl << "}";
	}
}

void write_metadata_jsonl_line(std::ostream &out, libcamera::ControlList &metadata)
{
	const libcamera::ControlIdMap *id_map = metadata.idMap();
	out << "{";
	bool first_done = false;
	for (auto const &[id, val] : metadata)
	{
		std::string arg_quote = (val.toString().find('/') != std::string::npos) ? "\"" : "";
		out << (first_done ? "," : "") << "\"" << id_map->at(id)->name() << "\": " << arg_quote
		    << val.toString() << arg_quote;
		first_done = true;
	}
	out << "}\n";
}

void stop_metadata_output(std::streambuf *buf, std::string fmt)
{
	std::ostream out(buf);
	if (fmt == "json")
		out << std::endl << "]" << std::endl;
}
