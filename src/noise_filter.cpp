#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string.h>

#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <vector>

using namespace std;

#define default_window_size 31

#define time_sliding_window_size 5

int window_size = default_window_size;

class Filter {
   private:
    ros::NodeHandle node_handle;
    ros::Subscriber sub;
    ros::Publisher pub;

    vector<vector<double>> queue;

    void grouping(int start_index, int end_index, vector<int>& target) {
        target.clear();
        int currentIndex = start_index;
        while (currentIndex != end_index) {
            target.push_back(currentIndex);
            currentIndex += 1;
            if (currentIndex >= 720) {
                currentIndex = 0;
            }
        }
    }

    int window_lower_limit, window_upper_limit, positive_gain, negative_gain;

   public:
    Filter() {
        sub = node_handle.subscribe<sensor_msgs::LaserScan>("/scan", 1, &Filter::scanCallback, this);
        pub = node_handle.advertise<sensor_msgs::LaserScan>("/scan/noise_filter", 1);
        node_handle.param<int>("window_lower_limit", window_lower_limit, 31);
        node_handle.param<int>("window_upper_limit", window_upper_limit, 101);
        node_handle.param<int>("negative_gain", negative_gain, 3);
        node_handle.param<int>("positive_gain", positive_gain, 20);
    }

    int number_of_flips(double var_array[time_sliding_window_size]) {
        int count = 0;
        for (int i = 0; i < time_sliding_window_size - 1; i++) {
            if (var_array[i] == 0 && var_array[i + 1] != 0 || var_array[i] != 0 && var_array[i + 1] == 0) {
                count++;
            }
        }
        return count;
    }

    void neighboring_filter(vector<double>& average_array, const sensor_msgs::LaserScan::ConstPtr& scan) {
        average_array.clear();
        int half_window_size = floor(window_size / 2);
        for (int i = 0; i < 720; i++) {
            if (scan->ranges[i] > 0) {
                average_array.push_back(scan->ranges[i]);
            } else {
                double total = 0, no_zero_elements = 0;
                for (int j = i - half_window_size; j <= i + half_window_size; j++) {
                    int index = j;
                    if (j < 0) {
                        index += 720;
                    } else if (j >= 720) {
                        index -= 720;
                    }

                    total += scan->ranges[index];
                    if (scan->ranges[index] > 0) {
                        no_zero_elements++;
                    }
                }

                average_array.push_back(no_zero_elements == 0 ? 0 : total / no_zero_elements);
            }
        }
    }

    void sequential_filter(vector<float>& array) {
        static int weight[] = {1, 2, 3, 4, 8};
        array.clear();
        array.resize(720);
        for (int i = 0; i < queue.size(); i++) {
            for (int j = 0; j < queue[i].size(); j++) {
                array[j] += queue[i][j] * weight[i];
            }
        }
        double total_number_of_flips = 0;
        for (int i = 0; i < 720; i++) {
            double time_slice_array[time_sliding_window_size];
            for (int j = 0; j < time_sliding_window_size; j++) {
                time_slice_array[j] = queue[j][i];
            }

            total_number_of_flips += number_of_flips(time_slice_array);
        }

        if (total_number_of_flips / 720 > 0.08) {
            window_size += positive_gain;
        } else {
            window_size -= negative_gain;
        }

        if (window_size < window_lower_limit) {
            window_size = window_lower_limit;
        } else if (window_size > window_upper_limit) {
            window_size = window_upper_limit;
        }

        for (int i = 0; i < 720; i++) {
            array[i] /= 18;
        }

        queue.erase(queue.begin());
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        vector<double> filter_array;
        neighboring_filter(filter_array, scan);
        queue.push_back(move(filter_array));
        if (queue.size() == time_sliding_window_size) {
            sensor_msgs::LaserScan msg;
            msg.header.stamp = scan->header.stamp;
            msg.header.frame_id = scan->header.frame_id;
            msg.angle_min = scan->angle_min;
            msg.angle_max = scan->angle_max;
            msg.angle_increment = scan->angle_increment;
            msg.scan_time = scan->scan_time;
            msg.time_increment = scan->time_increment;
            msg.range_min = scan->range_min;
            msg.range_max = scan->range_max;
            msg.intensities =  scan->intensities;
            sequential_filter(msg.ranges);
            pub.publish(msg);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "noise_filter");
    Filter filter;

    ros::spin();

    return 0;
}