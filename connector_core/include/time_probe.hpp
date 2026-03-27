#ifndef CPU_TIME_PROBE_HPP
#define CPU_TIME_PROBE_HPP

#include <chrono>
#include <fstream>
#include <string>
#include <time.h>

class CpuTimeProbe
{
public:
    CpuTimeProbe()
        : elapsed_time(0.0f), save_data(false)
    {
    }

    explicit CpuTimeProbe(std::string out_file)
        : elapsed_time(0.0f), save_data(true), out_file(std::move(out_file))
    {
        file.open(this->out_file.c_str(), std::ios::out);
        file << "Time[us]" << std::endl;
        file.close();
    }

    void start()
    {
        start_time = clock();
    }

    void stop()
    {
        end_time = clock();
        elapsed_time = float(end_time - start_time) / CLOCKS_PER_SEC;
        if (save_data)
        {
            file.open(out_file.c_str(), std::ios_base::app);
            file << get_elapsed_time() << std::endl;
            file.close();
        }
    }

    float get_elapsed_time()
    {
        return elapsed_time;
    }

private:
    clock_t start_time{};
    clock_t end_time{};
    float elapsed_time;
    bool save_data;
    std::string out_file;
    std::ofstream file;
};

class WallTimeProbe
{
public:
    WallTimeProbe()
        : elapsed_time(0), save_data(false)
    {
    }

    explicit WallTimeProbe(std::string out_file)
        : elapsed_time(0), save_data(true), out_file(std::move(out_file))
    {
        file.open(this->out_file.c_str(), std::ios::out);
        file << "Time[us]" << std::endl;
        file.close();
    }

    void start()
    {
        start_time = std::chrono::system_clock::now();
    }

    void stop()
    {
        end_time = std::chrono::system_clock::now();
        elapsed_time =
            std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        if (save_data)
        {
            file.open(out_file.c_str(), std::ios_base::app);
            file << get_elapsed_time() << std::endl;
            file.close();
        }
    }

    uint64_t get_elapsed_time()
    {
        return elapsed_time;
    }

private:
    std::chrono::system_clock::time_point start_time;
    std::chrono::system_clock::time_point end_time;
    int64_t elapsed_time;
    bool save_data;
    std::string out_file;
    std::ofstream file;
};

#endif  // CPU_TIME_PROBE_HPP
