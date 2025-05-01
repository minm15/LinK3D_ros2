// src/RosbagMain.cpp

#include "LinK3D_extractor.h"
#include <geometry_msgs/msg/point.hpp>
#include <mutex>
#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl_conversions/pcl_conversions.h>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std;
using namespace LinK3D_SLAM;

//── Aggregation keypoints RANSAC ───────────────────────────────────────────────
void ransac(const vector<pcl::PointXYZ>& vpCurPt, const vector<pcl::PointXYZ>& vpLastPt,
            const vector<pair<int, int>>& vMatchedIndex,
            vector<pair<int, int>>& vTrueMatchedIndex) {
    if (vMatchedIndex.empty())
        return;

    auto source = make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto target = make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::CorrespondencesPtr corrsPtr(new pcl::Correspondences());

    for (size_t i = 0; i < vMatchedIndex.size(); ++i) {
        source->push_back(vpCurPt[vMatchedIndex[i].first]);
        target->push_back(vpLastPt[vMatchedIndex[i].second]);
        corrsPtr->push_back(pcl::Correspondence(i, i, 0));
    }

    pcl::Correspondences corrs;
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rej;
    rej.setInputSource(source);
    rej.setInputTarget(target);
    rej.setInlierThreshold(0.5);
    rej.getRemainingCorrespondences(*corrsPtr, corrs);

    for (auto& c : corrs) {
        vTrueMatchedIndex.emplace_back(vMatchedIndex[c.index_query]);
    }
}

//── Edge keypoints RANSAC ────────────────────────────────────────────────────
void ransacForEdgePt(const MatPt& matchedEdgePt,
                     vector<pair<PointXYZSCA, PointXYZSCA>>& vTrueMatchedEdgePoint) {
    if (matchedEdgePt.empty())
        return;

    auto source = make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    auto target = make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::CorrespondencesPtr corrsPtr(new pcl::Correspondences());

    for (size_t i = 0; i < matchedEdgePt.size(); ++i) {
        const auto& mp = matchedEdgePt[i];
        pcl::PointXYZI src{mp[0].x, mp[0].y, mp[0].z};
        pcl::PointXYZI dst{mp[1].x, mp[1].y, mp[1].z};
        source->push_back(src);
        target->push_back(dst);
        corrsPtr->push_back(pcl::Correspondence(i, i, 0));
    }

    pcl::Correspondences corrs;
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> rej;
    rej.setInputSource(source);
    rej.setInputTarget(target);
    rej.setInlierThreshold(0.5);
    rej.getRemainingCorrespondences(*corrsPtr, corrs);

    for (auto& c : corrs) {
        const auto& mp = matchedEdgePt[c.index_query];
        vTrueMatchedEdgePoint.emplace_back(mp[0], mp[1]);
    }
}

static queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> cloudBuffer;
static mutex mBuf;

void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    lock_guard<mutex> lk(mBuf);
    cloudBuffer.push(msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("link3d_rosbag");

    // param
    int scan_line = 32;
    node->declare_parameter("scan_line", scan_line);
    node->get_parameter("scan_line", scan_line);

    // Publishers
    auto pubFullCloud1 = node->create_publisher<sensor_msgs::msg::PointCloud2>("full_cloud1", 10);
    auto pubFullCloud2 = node->create_publisher<sensor_msgs::msg::PointCloud2>("full_cloud2", 10);
    auto pubKeyPoint = node->create_publisher<sensor_msgs::msg::PointCloud2>("key_point", 10);
    auto pubMarker = node->create_publisher<visualization_msgs::msg::Marker>("marker", 10);

    // Subscriber
    auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_points", 10,
                                                                        laserCloudHandler);

    // LiK3D extractor
    auto extractor = make_shared<LinK3D_Extractor>(scan_line, 0.1f, 0.4f, 0.3f, 0.3f, 12, 4, 3);

    // cache the last frame
    auto currentCloud = make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto lastCloud = make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    vector<pcl::PointXYZ> lastKeyPts;
    cv::Mat lastDesc;
    vector<int> lastIdx;
    MatPt lastHighSmooth;

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.pretranslate(Eigen::Vector3d(0, 0, 30));

    size_t count = 0;
    rclcpp::Rate rate(10.0);

    while (rclcpp::ok()) {
        if (!cloudBuffer.empty()) {
            {
                lock_guard<mutex> lk(mBuf);
                pcl::fromROSMsg(*cloudBuffer.front(), *currentCloud);
                cloudBuffer.pop();
            }

            vector<pcl::PointXYZ> curKeyPts;
            cv::Mat curDesc;
            vector<int> curIdx;
            MatPt curClustered, curHighSmooth;
            (*extractor)(*currentCloud, curKeyPts, curDesc, curIdx, curClustered);
            extractor->filterLowSmooth(curClustered, curHighSmooth);

            if (count > 0) {
                // match
                vector<pair<int, int>> vMatched, vTrueMatch;
                extractor->matcher(curDesc, lastDesc, vMatched);
                MatPt matchedEdge;
                extractor->matchEdgePoints(curHighSmooth, lastHighSmooth, curIdx, lastIdx,
                                           matchedEdge, vMatched);

                // RANSAC
                ransac(curKeyPts, lastKeyPts, vMatched, vTrueMatch);
                vector<pair<PointXYZSCA, PointXYZSCA>> vTrueEdges;
                ransacForEdgePt(matchedEdge, vTrueEdges);

                // Marker publish
                visualization_msgs::msg::Marker ml;
                ml.header.frame_id = "map";
                ml.header.stamp = node->get_clock()->now();
                ml.type = visualization_msgs::msg::Marker::LINE_LIST;
                ml.scale.x = 0.07f;
                ml.color.r = 0.0;
                ml.color.g = 1.0;
                ml.color.b = 0.3;
                ml.color.a = 1.0;

                pcl::PointCloud<pcl::PointXYZ> allKP;
                for (auto& pr : vTrueEdges) {
                    const auto& p1s = pr.first;
                    const auto& p2s = pr.second;
                    Eigen::Vector3d p2t = T * Eigen::Vector3d(p2s.x, p2s.y, p2s.z);

                    geometry_msgs::msg::Point P1;
                    P1.x = p1s.x;
                    P1.y = p1s.y;
                    P1.z = p1s.z;
                    geometry_msgs::msg::Point P2;
                    P2.x = p2t.x();
                    P2.y = p2t.y();
                    P2.z = p2t.z();

                    ml.points.push_back(P1);
                    ml.points.push_back(P2);

                    allKP.push_back(pcl::PointXYZ(p1s.x, p1s.y, p1s.z));
                    allKP.push_back(pcl::PointXYZ(p2t.x(), p2t.y(), p2t.z()));
                }
                pubMarker->publish(ml);

                // key_point publish
                sensor_msgs::msg::PointCloud2 outK;
                pcl::toROSMsg(allKP, outK);
                outK.header.stamp = node->get_clock()->now();
                outK.header.frame_id = "map";
                pubKeyPoint->publish(outK);

                // full_cloud1 publish
                sensor_msgs::msg::PointCloud2 outC1;
                pcl::toROSMsg(*currentCloud, outC1);
                outC1.header.stamp = node->get_clock()->now();
                outC1.header.frame_id = "map";
                pubFullCloud1->publish(outC1);

                // full_cloud2 publish
                pcl::PointCloud<pcl::PointXYZ> lastT;
                for (auto& pt : lastCloud->points) {
                    auto tp = T * Eigen::Vector3d(pt.x, pt.y, pt.z);
                    lastT.push_back(pcl::PointXYZ(tp.x(), tp.y(), tp.z()));
                }
                sensor_msgs::msg::PointCloud2 outC2;
                pcl::toROSMsg(lastT, outC2);
                outC2.header.stamp = node->get_clock()->now();
                outC2.header.frame_id = "map";
                pubFullCloud2->publish(outC2);
            }

            // update the data from last frame
            *lastCloud = *currentCloud;
            lastKeyPts = curKeyPts;
            lastDesc = curDesc.clone();
            lastIdx = curIdx;
            lastHighSmooth = curHighSmooth;
            count++;
        }

        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
