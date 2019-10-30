// match_template.cpp:
// Match templates to recognize digits inside the green circles of the image

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

#include <opencv2/opencv.hpp>

const double MIN_AREA_SIZE = 100;

std::string template_folder = "../imgs/template/";

void processImage(const std::string& filename)
{
  // Load image from file
  cv::Mat img = cv::imread(filename.c_str());
  if(img.empty()) {
    throw std::runtime_error("Failed to open the file " + filename);
  }
  
  // Display original image
  cv::imshow("Original img", img); // Displays the original image in a window named "Original img".
  //waitKey displays the image and pause the execution of the program for specified milliseconds.
  // Otherwise, it won’t display the image. For example, waitKey(0) will display the window infinitely
  // until any keypress (it is suitable for image display). waitKey(25) will display a frame for 25 ms, 
  // after which display will be automatically closed. (If you put it in a loop to read videos, it will 
  // display the video frame-by-frame)
  cv::waitKey(20);  

  // Convert color space from BGR to HSV
  // NOTE: OpenCv color convention for historical reason is BGR and not RGB. Just a convention!
  // cvtColor can convert between several colorspace check at https://docs.opencv.org/master/d8/d01/group__imgproc__color__conversions.html#gga4e0972be5de079fed4e3a10e24ef5ef0a353a4b8db9040165db4dacb5bcefb6ea  
  cv::Mat hsv_img;
  cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);
    
  // Find green regions
  cv::Mat green_mask;
  // store a binary image in green_mask where the white pixel are those contained in HSV rage (45,40,40) --> (75,255,255)  
  cv::inRange(hsv_img, cv::Scalar(45, 40, 40), cv::Scalar(75, 255, 255), green_mask);
  
  // Apply some filtering
  // Create the kernel of the filter i.e. a rectanble with dimension 3x3
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));
  // Dilate using the generated kernel
  cv::dilate(green_mask, green_mask, kernel);
  // Erode using the generated kernel
  cv::erode(green_mask,  green_mask, kernel);
  
  // Display the binary image
  cv::imshow("GREEN_filter", green_mask);
  cv::waitKey(500);    //  wait 500 ms
  
  // Find contours
  std::vector<std::vector<cv::Point>> contours, contours_approx;  
  
  // Create an image which we can modify not changing the original image!
  cv::Mat contours_img;
  contours_img = img.clone();

  // Finds contours in a binary image.
  cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);  
    
  // create an array of rectangle (i.e. bounding box containing the green area contour)  
  std::vector<cv::Rect> boundRect(contours.size());
  for (int i=0; i<contours.size(); ++i)
  {
    double area = cv::contourArea(contours[i]);
    if (area < MIN_AREA_SIZE) continue; // filter too small contours to remove false positives

    std::vector<cv::Point> approx_curve;
    approxPolyDP(contours[i], approx_curve, 10, true);
    if(approx_curve.size() < 6) continue; //fitler out the gate 
    contours_approx = {approx_curve};

    // Draw the contours on image with a line color of BGR=(0,170,220) and a width of 3
    drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);

    // find the bounding box of the green blob approx curve
    boundRect[i] = boundingRect(cv::Mat(approx_curve)); 
  }


  // Display the image 
  cv::imshow("Original", contours_img);
  cv::waitKey(20);
     
  cv::Mat green_mask_inv;

  // Init a matrix specify its dimension (img.rows, img.cols), default color(255,255,255) 
  // and elemet type (CV_8UC3).
  cv::Mat filtered(img.rows, img.cols, CV_8UC3, cv::Scalar(255,255,255));

  // generate binary mask with inverted pixels w.r.t. green mask -> black numbers are part of this mask
  cv::bitwise_not(green_mask, green_mask_inv); 

  // Show the inverted mask. The green area should be black here!
  cv::imshow("Numbers", green_mask_inv);
  cv::waitKey(20);
  
  // Load digits template images
  std::vector<cv::Mat> templROIs;
  for (int i=1; i<=5; ++i) {
    auto num_template = cv::imread(template_folder + std::to_string(i) + ".png");
    // mirror the template, we want them to have the same shape of the number that we
    // have in the unwarped ground image
    cv::flip(num_template, num_template, 1); 

    // Show the loaded template!
    cv::imshow("Loaded template " + std::to_string(i) ,num_template);
    cv::waitKey(20);

    // Store the template in templROIs (vector of mat)
    templROIs.emplace_back(num_template);
  }  
  
  img.copyTo(filtered, green_mask_inv);   // create copy of image without green shapes


  // create a 3x3 recttangular kernel for img filtering
  kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2*2) + 1, (2*2)+1));

  // For each green blob in the original image containing a digit
  for (int i=0; i<boundRect.size(); ++i)
  {
    
    // Constructor of mat, we pass the original image and the coordinate to copy and we obtain
    // an image pointing to that subimage
    cv::Mat processROI(filtered, boundRect[i]); // extract the ROI containing the digit
    
    if (processROI.empty()) continue;
    
    // The size of the number in the Template image should be similar to the dimension
    // of the number in the ROI
    cv::resize(processROI, processROI, cv::Size(200, 200)); // resize the ROI 
    cv::threshold( processROI, processROI, 100, 255, 0 );   // threshold and binarize the image, to suppress some noise
    
    // Apply some additional smoothing and filtering
    cv::erode(processROI, processROI, kernel);
    cv::GaussianBlur(processROI, processROI, cv::Size(5, 5), 2, 2);
    cv::erode(processROI, processROI, kernel);
    
    // Show the actual image used for the template matching
    cv::imshow("ROI", processROI);
      
    // Find the template digit with the best matching
    double maxScore = 0;
    int maxIdx = -1;  
    for (int j=0; j<templROIs.size(); ++j) {
      for(int a = 0; a<30; a++){
        cv::Mat result;
        cv::Point2f src_center(templROIs[j].cols/2.0F, templROIs[j].rows/2.0F);
        cv::Mat rot_mat = getRotationMatrix2D(src_center, a*12, 1.0);
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
    
    // Display the best fitting number
    std::cout << "Best fitting template: " << maxIdx + 1 << std::endl;    
    cv::waitKey(20);
  }
}

int main(int argc, char* argv[])
{
  if (argc != 2) {
    std::cout << "Usage: " << argv[0] << " <image>" << std::endl;
    return 0;
  }
  processImage(argv[1]);
  return 0;
}
