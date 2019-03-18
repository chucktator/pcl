/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Marco Paladini <marco.paladini@ocado.com>
 *  Date: March 2014
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>

/**
 * @brief The CameraPoseProcessor class is an interface to extract
 * camera pose data generated by the pcl_kinfu_app program.
 * Use the CameraPoseWriter implementation if writing the camera
 * pose to a file is all you need, or provide an alternative implementation.
 */
class CameraPoseProcessor
{
  public:
    virtual ~CameraPoseProcessor () {}

    /// process the camera pose, this method is called at every frame.
    virtual void
    processPose (const Eigen::Affine3f &pose)=0;
};

/**
 * @brief CameraPoseWriter writes all camera poses computed by
 * the KinfuTracker to a file on disk.
 *
 */
class CameraPoseWriter : public CameraPoseProcessor
{
  std::string output_filename_;
  std::ofstream out_stream_;
  public:
    /**
       * @param output_filename name of file to write
       */
    CameraPoseWriter (const std::string &output_filename) :
      output_filename_ (output_filename)
    {
      out_stream_.open (output_filename_.c_str () );
    }

    ~CameraPoseWriter ()
    {
      if (out_stream_.is_open ())
      {
        out_stream_.close ();
        std::cout << "wrote camera poses to file " << output_filename_ << std::endl;
      }
    }

    void
    processPose (const Eigen::Affine3f &pose) override
    {
      if (out_stream_.good ())
      {
        // convert 3x4 affine transformation to quaternion and write to file
        Eigen::Quaternionf q (pose.rotation ());
        Eigen::Vector3f t (pose.translation ());
        // write translation , quaternion in a row
        out_stream_ << t[0] << "," << t[1] << "," << t[2]
                    << "," << q.w () << "," << q.x ()
                    << "," << q.y ()<< ","  << q.z () << std::endl;
      }
    }

};
