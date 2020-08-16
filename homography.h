
#include "opencv2/highgui.hpp" 
#include "opencv2/imgcodecs.hpp" 
#include "opencv2/imgproc.hpp" 
#include <iostream>
#include <fstream>

void setup_transformation_matrix(void);

struct Obj_pixels {
    int iSource_id;
    int iNumber_of_person;
    int iImage_x[50];
    int iImage_y[50];
    int oMap_x[50];
    int oMap_y[50];
};


Obj_pixels test(Obj_pixels Obj_p);
void setup_transformation_matrix(void);
cv::Point2f transformPoint(cv::Point2f current, cv::Mat transformation);
Obj_pixels TransformImage2Map(Obj_pixels Obj_p);


