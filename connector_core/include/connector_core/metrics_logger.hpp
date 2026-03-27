#pragma once

#include <chrono>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace dancers {
namespace metrics {

inline uint64_t now_us()
{
    using namespace std::chrono;
    return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}

inline std::string csv_escape(const std::string& s)
{
    bool need_quotes = false;
    for (char c : s)
    {
        if (c == ',' || c == '"' || c == '\n' || c == '\r')
        {
            need_quotes = true;
            break;
        }
    }
    if (!need_quotes) return s;

    std::string out;
    out.reserve(s.size() + 2);
    out.push_back('"');
    for (char c : s)
    {
        if (c == '"') out.push_back('"');
        out.push_back(c);
    }
    out.push_back('"');
    return out;
}

class CsvMetricsLogger
{
public:
    CsvMetricsLogger(std::string file_path,
                     std::vector<std::string> header,
                     size_t flush_every_n = 50,
                     bool append = false)
        : file_path_(std::move(file_path)),
          header_(std::move(header)),
          flush_every_n_(flush_every_n),
          lines_since_flush_(0)
    {
        std::filesystem::path p(file_path_);
        if (p.has_parent_path())
        {
            std::filesystem::create_directories(p.parent_path());
        }

        std::ios::openmode mode = std::ios::out;
        if (append) mode |= std::ios::app;
        else mode |= std::ios::trunc;
        file_.open(file_path_, mode);

        try
        {
            if (!std::filesystem::exists(p) || std::filesystem::file_size(p) == 0)
            {
                write_header_();
                file_.flush();
            }
        }
        catch (...)
        {
        }
    }

    ~CsvMetricsLogger()
    {
        std::lock_guard<std::mutex> lk(m_);
        if (file_.is_open())
        {
            file_.flush();
            file_.close();
        }
    }

    CsvMetricsLogger(const CsvMetricsLogger&) = delete;
    CsvMetricsLogger& operator=(const CsvMetricsLogger&) = delete;

    template <typename... Args>
    void log_row(Args&&... args)
    {
        std::lock_guard<std::mutex> lk(m_);
        if (!file_.is_open()) return;

        std::vector<std::string> cells;
        cells.reserve(sizeof...(Args));
        (cells.emplace_back(to_cell_(std::forward<Args>(args))), ...);

        for (size_t i = 0; i < cells.size(); ++i)
        {
            if (i) file_ << ",";
            file_ << cells[i];
        }
        file_ << "\n";

        if (++lines_since_flush_ >= flush_every_n_)
        {
            file_.flush();
            lines_since_flush_ = 0;
        }
    }

private:
    void write_header_()
    {
        if (header_.empty()) return;
        for (size_t i = 0; i < header_.size(); ++i)
        {
            if (i) file_ << ",";
            file_ << csv_escape(header_[i]);
        }
        file_ << "\n";
    }

    static std::string to_cell_(const std::string& v) { return csv_escape(v); }
    static std::string to_cell_(const char* v)
    {
        return csv_escape(v ? std::string(v) : std::string(""));
    }

    template <typename T>
    static std::string to_cell_(const T& v)
    {
        std::ostringstream oss;
        if constexpr (std::is_floating_point_v<T>)
        {
            oss.setf(std::ios::fixed);
            oss.precision(6);
            oss << v;
            return oss.str();
        }
        else if constexpr (std::is_same_v<T, bool>)
        {
            return v ? "1" : "0";
        }
        else
        {
            oss << v;
            return oss.str();
        }
    }

    std::string file_path_;
    std::vector<std::string> header_;
    size_t flush_every_n_;
    size_t lines_since_flush_;
    std::ofstream file_;
    std::mutex m_;
};

}  // namespace metrics
}  // namespace dancers
