#include "include/find_victim.hpp"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

int get_victim_number(cv::Rect singleRect, cv::Mat img, cv::Mat &showImage, const std::string &config_folder) {
    cv::Mat hsv_img;
    cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);

    cv::Mat green_mask;
    cv::inRange(hsv_img, cv::Scalar(40, 30, 50), cv::Scalar(85, 255, 180), green_mask);
    cv::Mat green_mask_inv;

    // Init a matrix specify its dimension (img.rows, img.cols), default color(255,255,255)
    // and elemet type (CV_8UC3).
    cv::Mat filtered(img.rows, img.cols, CV_8UC3, cv::Scalar(255, 255, 255));

    // generate binary mask with inverted pixels w.r.t. green mask -> black numbers are part of this mask
    cv::bitwise_not(green_mask, green_mask_inv);

    cv::imshow("Original", green_mask_inv);
    cv::waitKey(10);

    // Load digits template images
    std::vector<cv::Mat> templROIs;

    for (int i = 1; i <= 5; ++i) {
        auto num_template = cv::imread(config_folder + "template/" + std::to_string(i) + ".png");
        // mirror the template, we want them to have the same shape of the number that we
        // have in the unwarped ground image
        cv::flip(num_template, num_template, 1);

        // Store the template in templROIs (vector of mat)
        templROIs.emplace_back(num_template);
    }

    img.copyTo(filtered, green_mask_inv);   // create copy of image without green shapes

    // create a 3x3 recttangular kernel for img filtering
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1 * 2) + 1, (1 * 2) + 1));
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2 * 2) + 1, (2 * 2) + 1));

    //Process single rect

    // Constructor of mat, we pass the original image and the coordinate to copy and we obtain
    // an image pointing to that subimage
    cv::Mat processROI(filtered, singleRect); // extract the ROI containing the digit

    if (processROI.empty()) return -1;

    // The size of the number in the Template image should be similar to the dimension
    // of the number in the ROI
    cv::resize(processROI, processROI, cv::Size(200, 200)); // resize the ROI
    cv::threshold(processROI, processROI, 100, 255, 0);   // threshold and binarize the image, to suppress some noise

    // Apply some additional smoothing and filtering
    /*cv::erode(processROI, processROI, kernel);
    cv::GaussianBlur(processROI, processROI, cv::Size(5, 5), 2, 2);
    cv::erode(processROI, processROI, kernel);*/

    // Show the actual image used for the template matching
    //cv::imshow("ROI", processROI);

    // Find the template digit with the best matching
    double maxScore = 0;
    int maxIdx = -1;
    for (int j = 0; j < templROIs.size(); ++j) {
        for (int a = 0; a < 72; a++) {
            cv::Mat result;
            cv::Point2f src_center(templROIs[j].cols / 2.0F, templROIs[j].rows / 2.0F);
            cv::Mat rot_mat = getRotationMatrix2D(src_center, a * 5, 1.0);
            cv::Mat dst;
            cv::warpAffine(templROIs[j], dst, rot_mat, templROIs[j].size());

            // Match the ROI with the templROIs j-th
            cv::matchTemplate(processROI, dst, result, cv::TM_CCOEFF);
            double score;
            cv::minMaxLoc(result, nullptr, &score);

            // Compare the score with the others, if it is higher save this as the best match!
            if (score > maxScore) {
                maxScore = score;
                maxIdx = j;
            }
        }
    }

    cv::Point point0 = cv::Point(singleRect.x, singleRect.y);
    cv::putText(showImage, std::to_string(maxIdx + 1), point0, cv::FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv::LINE_AA);

    //std::cout << "Best fitting template: " << maxIdx + 1 << std::endl;
    cv::waitKey(20);

    return maxIdx + 1;
}