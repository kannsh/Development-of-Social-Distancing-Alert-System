#include "homography.h"

cv::Mat MatImage2World;

void setup_transformation_matrix(void)
{   
    // Read config.txt file (It contains image & world coordinates input by user)
    std::fstream configfile;
    configfile.open("//home//student//Desktop//config.txt");
    
    // Declare variables for coordinate x and y value
    int imagecorner_x1, imagecorner_y1, imagecorner_x2, imagecorner_y2, imagecorner_x3, imagecorner_y3, imagecorner_x4, imagecorner_y4;
    int worldcorner_x1, worldcorner_y1, worldcorner_x2, worldcorner_y2, worldcorner_x3, worldcorner_y3, worldcorner_x4, worldcorner_y4;
    
    // File read in the values and output to assigned variable 
    configfile >> imagecorner_x1 >> imagecorner_y1 >> imagecorner_x2 >> imagecorner_y2 >> imagecorner_x3 >> imagecorner_y3 >> imagecorner_x4 >> imagecorner_y4                                           
               >> worldcorner_x1 >> worldcorner_y1 >> worldcorner_x2 >> worldcorner_y2 >> worldcorner_x3 >> worldcorner_y3 >> worldcorner_x4 >> worldcorner_y4;

    // Display imagecorner coordinate values
    std::cout << "\n[Imagecorner coordinates]" << std::endl;
    std::cout << "Imagecorner_x1: " << imagecorner_x1 << "   " << "Imagecorner_y1: " << imagecorner_y1 << std::endl;
    std::cout << "Imagecorner_x2: " << imagecorner_x2 << "  " << "Imagecorner_y2: " << imagecorner_y2 << std::endl;
    std::cout << "Imagecorner_x3: " << imagecorner_x3 << "  " << "Imagecorner_y3: " << imagecorner_y3 << std::endl;
    std::cout << "Imagecorner_x4: " << imagecorner_x4 << "   " << "Imagecorner_y4: " << imagecorner_y4 << std::endl;

    // Display worldcorner coordinate values
    std::cout << "\n[Worldcorner coordinates]" << std::endl;
    std::cout << "Worldcorner_x1: " << worldcorner_x1 << "     " << "Worldcorner_y1: " << worldcorner_y1 << std::endl;
    std::cout << "Worldcorner_x2: " << worldcorner_x2 << "  " << "Worldcorner_y2: " << worldcorner_y2 << std::endl;
    std::cout << "Worldcorner_x3: " << worldcorner_x3 << "  " << "Worldcorner_y3: " << worldcorner_y3 << std::endl;
    std::cout << "Worldcorner_x4: " << worldcorner_x4 << "     " << "Worldcorner_y4: " << worldcorner_y4 << std::endl;

    std::vector<cv::Point2f> imageLocs;
    imageLocs.push_back(cv::Point2f(imagecorner_x1, imagecorner_y1));
    imageLocs.push_back(cv::Point2f(imagecorner_x2, imagecorner_y2));
    imageLocs.push_back(cv::Point2f(imagecorner_x3, imagecorner_y3));
    imageLocs.push_back(cv::Point2f(imagecorner_x4, imagecorner_y4));

    std::vector<cv::Point2f> worldLocs; 
    
    worldLocs.push_back(cv::Point2f(worldcorner_x1, worldcorner_y1));
    worldLocs.push_back(cv::Point2f(worldcorner_x2, worldcorner_y2));
    worldLocs.push_back(cv::Point2f(worldcorner_x3, worldcorner_y3));
    worldLocs.push_back(cv::Point2f(worldcorner_x4, worldcorner_y4)); 

    MatImage2World = cv::getPerspectiveTransform(imageLocs, worldLocs); 
    
    // Close config.txt file after reading
    configfile.close();
}

cv::Point2f transformPoint(cv::Point2f current, cv::Mat transformation)
{
    cv::Point2f transformedPoint;
    transformedPoint.x = current.x * transformation.at<double>(0,0) + current.y * transformation.at<double>(0,1) + transformation.at<double>(0,2);
    transformedPoint.y = current.x * transformation.at<double>(1,0) + current.y * transformation.at<double>(1,1) + transformation.at<double>(1,2);
    float z = current.x * transformation.at<double>(2,0) + current.y * transformation.at<double>(2,1) + transformation.at<double>(2,2);
    transformedPoint.x /= z;
    transformedPoint.y /= z;
    return transformedPoint;
}


Obj_pixels TransformImage2Map(Obj_pixels Obj_p)
{
    cv::Point2f imagePoint = cv::Point2f(Obj_p.iImage_x[0],Obj_p.iImage_y[0]);  
    cv::Point2f tpoint = transformPoint(imagePoint, MatImage2World);       
    Obj_p.oMap_x[0]=tpoint.x;
    Obj_p.oMap_y[0]=tpoint.y;
    return (Obj_p);
}


















