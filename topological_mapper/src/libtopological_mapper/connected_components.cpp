/**
 * \file  connected_components.h
 * \brief  Implementation for Connected components. Uses cvblobslib internally.
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 *
 * Copyright (c) 2013, UT Austin

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *
 * $ Id: 03/04/2013 12:28:10 PM piyushk $
 *
 **/

#include <topological_mapper/connected_components.h>
#include <topological_mapper/map_loader.h> // for MAP_IDX
#include <cvblobs/BlobResult.h>

namespace topological_mapper {

  /**
   * \brief  Given an image map containing obstacles and demarcating
   *         critical lines, the constructor runs a critical components
   *         algorithm to find critical regions.
   * \param  image map with obstacles and critical lines 
   * \return  
   */
  ConnectedComponents::ConnectedComponents (cv::Mat& image,
      std::vector<int32_t>& component_map) {

    IplImage ipl_image = (IplImage)image;
    IplImage* image_ptr = cvCloneImage(&ipl_image);
    CBlobResult blobs(image_ptr, NULL, 0);
    number_components_ = blobs.GetNumBlobs();

    // Initialize vector to all zeros 
    for (size_t i = 0; i < component_map.size(); ++i) {
      component_map[i] = -1;
    }

    // Draw individual components onto image
    for (size_t t = 0; t < number_components_; ++t) {

      // Draw this component on to an image
      CBlob blob(blobs.GetBlob(t));
      IplImage *blob_image = 
        cvCreateImage(cvSize(image.cols, image.rows), IPL_DEPTH_8U, 1);
      cvSetZero(blob_image);
      blob.FillBlob(blob_image, cvScalar(255));

      // Read the image and fill the std::vector
      int step = blob_image->widthStep / sizeof(uchar);
      uchar* data = (uchar *) blob_image->imageData;
      for (int j = 0; j < image.rows; ++j) {
        for (int i = 0; i < image.cols; ++i) {
          if (data[j * step + i] == 255) {
            size_t map_idx = MAP_IDX(image.cols, i, j);
            component_map[map_idx] = t;
          }
        }
      }

      cvReleaseImage(&blob_image);
    }

    cvReleaseImage(&image_ptr);

  }

  /**
   * \brief Returns the number of components obtained by running the
   *        critical components algorithm.
   */
  size_t ConnectedComponents::getNumberComponents() {
    return number_components_;
  }

} /* topological_mapper */
