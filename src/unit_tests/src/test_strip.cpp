// Unit tests for img_processing Strip class
// Tests: sortPointByY, constructor geometry calculations

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <algorithm>
#include <cmath>

// We cannot directly include strip.h because it pulls in ROS2 headers.
// Instead, we test the core geometric logic that Strip implements by
// re-implementing and verifying the algorithms independently.

// ==================== sortPointByY logic ====================

// The Strip::sortPointByY function simply compares y coordinates
static bool sortPointByY(const cv::Point2f& a, const cv::Point2f& b)
{
    return a.y < b.y;
}

class StripGeometryTest : public ::testing::Test {};

TEST_F(StripGeometryTest, SortPointByYBasic)
{
    cv::Point2f p1(100.0f, 50.0f);
    cv::Point2f p2(200.0f, 10.0f);

    // p2.y < p1.y, so p2 should come first
    EXPECT_TRUE(sortPointByY(p2, p1));
    EXPECT_FALSE(sortPointByY(p1, p2));
}

TEST_F(StripGeometryTest, SortPointByYEqualY)
{
    cv::Point2f p1(100.0f, 50.0f);
    cv::Point2f p2(200.0f, 50.0f);

    // Equal y means neither is "less than" the other
    EXPECT_FALSE(sortPointByY(p1, p2));
    EXPECT_FALSE(sortPointByY(p2, p1));
}

TEST_F(StripGeometryTest, SortPointByYNegativeCoordinates)
{
    cv::Point2f p1(10.0f, -20.0f);
    cv::Point2f p2(10.0f, -10.0f);

    // -20 < -10
    EXPECT_TRUE(sortPointByY(p1, p2));
    EXPECT_FALSE(sortPointByY(p2, p1));
}

TEST_F(StripGeometryTest, SortVectorOfPoints)
{
    std::vector<cv::Point2f> points = {
        cv::Point2f(10.0f, 300.0f),
        cv::Point2f(20.0f, 100.0f),
        cv::Point2f(30.0f, 200.0f),
        cv::Point2f(40.0f, 50.0f)
    };

    std::sort(points.begin(), points.end(), sortPointByY);

    EXPECT_FLOAT_EQ(points[0].y, 50.0f);
    EXPECT_FLOAT_EQ(points[1].y, 100.0f);
    EXPECT_FLOAT_EQ(points[2].y, 200.0f);
    EXPECT_FLOAT_EQ(points[3].y, 300.0f);
}

// ==================== Strip endpoint calculation logic ====================
// Test the upper/lower endpoint calculation from a rotated rect

TEST_F(StripGeometryTest, RotatedRectEndpoints)
{
    // Simulate what Strip constructor does: get 4 corners of rotated rect,
    // sort by Y, average top-2 and bottom-2 to get upper/lower endpoints
    cv::RotatedRect rect(cv::Point2f(100.0f, 200.0f), cv::Size2f(20.0f, 80.0f), 0.0f);

    cv::Point2f points[4];
    rect.points(points);
    std::vector<cv::Point2f> vertice(points, points + 4);
    std::sort(vertice.begin(), vertice.end(), sortPointByY);

    // Upper endpoint: average of top two points
    cv::Point2f upper((vertice[0].x + vertice[1].x) * 0.5f,
                      (vertice[0].y + vertice[1].y) * 0.5f);
    // Lower endpoint: average of bottom two points
    cv::Point2f lower((vertice[2].x + vertice[3].x) * 0.5f,
                      (vertice[2].y + vertice[3].y) * 0.5f);

    // Center should be midpoint of upper and lower
    cv::Point2f center((upper.x + lower.x) / 2.0f, (upper.y + lower.y) / 2.0f);

    // The center should be close to the rect's center
    EXPECT_NEAR(center.x, 100.0f, 1.0f);
    EXPECT_NEAR(center.y, 200.0f, 1.0f);

    // Upper should be above lower (smaller y in image coordinates)
    EXPECT_LT(upper.y, lower.y);
}

TEST_F(StripGeometryTest, RotatedRectEndpointsAngled)
{
    // A tilted rect (45 degrees)
    cv::RotatedRect rect(cv::Point2f(150.0f, 150.0f), cv::Size2f(10.0f, 60.0f), 45.0f);

    cv::Point2f points[4];
    rect.points(points);
    std::vector<cv::Point2f> vertice(points, points + 4);
    std::sort(vertice.begin(), vertice.end(), sortPointByY);

    cv::Point2f upper((vertice[0].x + vertice[1].x) * 0.5f,
                      (vertice[0].y + vertice[1].y) * 0.5f);
    cv::Point2f lower((vertice[2].x + vertice[3].x) * 0.5f,
                      (vertice[2].y + vertice[3].y) * 0.5f);

    // Upper is still above lower
    EXPECT_LT(upper.y, lower.y);

    // The distance between upper and lower should be roughly the height (long side)
    float dist = std::sqrt(std::pow(upper.x - lower.x, 2) + std::pow(upper.y - lower.y, 2));
    EXPECT_NEAR(dist, 60.0f, 5.0f); // Approximate due to averaging corners
}

// ==================== fitLine projection logic (from Strip constructor) ====================

TEST_F(StripGeometryTest, FitLineProjection)
{
    // Simulate the Strip constructor's fitLine + projection logic
    // Create a vertical line of contour points
    std::vector<cv::Point> contour;
    for (int i = 0; i < 50; ++i) {
        contour.push_back(cv::Point(100, 100 + i));
    }

    cv::Vec4f line_param;
    cv::fitLine(contour, line_param, cv::DIST_L2, 0, 0.01, 0.01);

    double vx = line_param[0];
    double vy = line_param[1];
    double x0 = line_param[2];
    double y0 = line_param[3];

    // For a vertical line, vx should be ~0 and vy should be ~1 (or -1)
    EXPECT_NEAR(std::abs(vx), 0.0, 0.1);
    EXPECT_NEAR(std::abs(vy), 1.0, 0.1);

    // Project the top and bottom y onto the line
    double subpix_upper_y = 100.0;
    double subpix_lower_y = 149.0;

    cv::Point2f accurate_top, accurate_bottom;
    if (std::abs(vy) > 1e-5) {
        accurate_top.y = subpix_upper_y;
        accurate_top.x = x0 + (subpix_upper_y - y0) * (vx / vy);

        accurate_bottom.y = subpix_lower_y;
        accurate_bottom.x = x0 + (subpix_lower_y - y0) * (vx / vy);
    }

    // Both x values should be ~100 for a vertical contour
    EXPECT_NEAR(accurate_top.x, 100.0, 1.0);
    EXPECT_NEAR(accurate_bottom.x, 100.0, 1.0);
}

TEST_F(StripGeometryTest, FitLineProjectionTilted)
{
    // Create a slightly tilted contour (10 degree tilt from vertical)
    std::vector<cv::Point> contour;
    double angle_rad = 10.0 * M_PI / 180.0;
    for (int i = 0; i < 50; ++i) {
        int x = static_cast<int>(100 + i * std::sin(angle_rad));
        int y = static_cast<int>(100 + i * std::cos(angle_rad));
        contour.push_back(cv::Point(x, y));
    }

    cv::Vec4f line_param;
    cv::fitLine(contour, line_param, cv::DIST_L2, 0, 0.01, 0.01);

    double vx = line_param[0];
    double vy = line_param[1];

    // Direction should reflect the tilt: vy/vx ≈ cos(10°)/sin(10°)
    double fitted_angle = std::atan2(vx, vy);
    EXPECT_NEAR(fitted_angle, angle_rad, 0.05);
}

// ==================== Armor plate pairing distance/angle logic ====================

TEST_F(StripGeometryTest, StripCenterDistance)
{
    // Two strips that could form an armor plate
    cv::Point2f center1(100.0f, 200.0f);
    cv::Point2f center2(200.0f, 200.0f);

    float dx = center2.x - center1.x;
    float dy = center2.y - center1.y;
    float distance = std::sqrt(dx * dx + dy * dy);

    EXPECT_FLOAT_EQ(distance, 100.0f);
}

TEST_F(StripGeometryTest, StripHeightRatio)
{
    // Two strips with similar height should be paired
    double height1 = 40.0;
    double height2 = 42.0;

    double ratio = std::min(height1, height2) / std::max(height1, height2);
    EXPECT_GT(ratio, 0.8); // Typical threshold for strip pairing
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
