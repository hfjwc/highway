#ifndef BYTETRACK_H
#define BYTETRACK_H

#include "lapjv.h"
#include "strack.h"

struct BoundingBox
{
    int   x;
    int   y;
    int   width;
    int   height;
    float score;
    int   class_id;
    void* userData;
};

struct bytetrack_params
{
    // detector:
    float conf_thresh = 0.25f;
    float nms_thresh  = 0.75f;
    // tracker:
    float track_thresh = 0.25f;
    float match_thresh = 0.7f;
    int   frame_rate   = 15;
    int   track_buffer = 10;
    int   min_box_area = 10;
};

///
/// \brief The BYTETracker class
/// 作为静态库进行调用，减少单个插件的库依赖
///

class BYTETracker
{
public:
    BYTETracker();
    ~BYTETracker();

    void init(const bytetrack_params& params);
    void clear();
    void clear_removed_stracks()
    {
        this->removed_stracks.clear();
    }

    void     update(STracks& output_stracks, const std::vector<BoundingBox>& objects);
    STracks& get_lost_stracks()
    {
        return lost_stracks;
    }
    STracks& get_removed_stracks()
    {
        return removed_stracks;
    }

    ///
    /// \brief delete_removed_stracks
    /// removed_stracks是增量的，为了避免内存溢出，需要定期清理
    ///
    void delete_removed_stracks()
    {
        removed_stracks.clear();
    }

private:
    void joint_stracks(STracks& tlista, STracks& tlistb, STracks& results);

    void sub_stracks(STracks& tlista, STracks& tlistb);

    void remove_duplicate_stracks(STracks& resa, STracks& resb, STracks& stracksa, STracks& stracksb);

    void linear_assignment(std::vector<std::vector<float>>& cost_matrix, int cost_matrix_size, int cost_matrix_size_size, float thresh, std::vector<std::vector<int>>& matches,
                           std::vector<int>& unmatched_a, std::vector<int>& unmatched_b);

    void iou_distance(const STracks& atracks, const STracks& btracks, std::vector<std::vector<float>>& cost_matrix);

    void ious(std::vector<std::vector<float>>& atlbrs, std::vector<std::vector<float>>& btlbrs, std::vector<std::vector<float>>& results);

    void lapjv(const std::vector<std::vector<float>>& cost, std::vector<int>& rowsol, std::vector<int>& colsol, bool extend_cost = false, float cost_limit = LONG_MAX, bool return_cost = true);

public:
    float track_thresh;
    float match_thresh;
    int   frame_rate;
    int   track_buffer;
    int   min_box_area;
    int   frame_id;
    int   max_time_lost;

    STracks tracked_stracks;
    STracks lost_stracks;
    STracks removed_stracks;

    std::shared_ptr<Kalman::KalmanFilter> kalman_filter;
};

#endif   // BYTETRACK_H
