#ifndef STRACK_H
#define STRACK_H

#include "kalmanfilter.h"
#include <chrono>
enum TrackState { New = 0,
    Tracked,
    Lost,
    Removed };

class STrack {
public:
    STrack(std::vector<float> tlwh_, float score, int class_id);
    STrack(std::vector<float> tlwh_, float score, int class_id, void* userData);
    ~STrack();

    std::vector<float> static tlbr_to_tlwh(std::vector<float>& tlbr);
    void static multi_predict(std::vector<std::shared_ptr<STrack>>& stracks,
        std::shared_ptr<Kalman::KalmanFilter> kalman_filter);
    void static_tlwh();
    void static_tlbr();
    std::vector<float> tlwh_to_xyah(std::vector<float> tlwh_tmp);
    std::vector<float> to_xyah();
    void mark_lost();
    void mark_removed();
    int next_id();
    int end_frame();

    void activate(std::shared_ptr<Kalman::KalmanFilter> kalman_filter, int frame_id);
    void re_activate(std::shared_ptr<Kalman::KalmanFilter> kalman_filter,
        std::shared_ptr<STrack> new_track, int frame_id,
        bool new_id = false);
    void update(std::shared_ptr<Kalman::KalmanFilter> kalman_filter,
        std::shared_ptr<STrack> new_track, int frame_id);

public:
    bool is_activated;
    int track_id;
    int state;

    std::vector<float> _tlwh;
    std::vector<float> tlwh;
    std::vector<float> tlbr;
    std::vector<float> det_tlwh;
    std::list<std::vector<float>> trajectory_blwh;
    int frame_id;
    int tracklet_len;
    int start_frame;
    std::chrono::high_resolution_clock::time_point start_time;
    std::chrono::high_resolution_clock::time_point lost_time;
    void* userData;
    cv::Mat mean;
    cv::Mat covariance;
    float score;
    int class_id;
};

using STracks = std::vector<std::shared_ptr<STrack>>;

#endif // STRACK_H
