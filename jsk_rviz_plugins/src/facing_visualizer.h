// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
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
 *********************************************************************/


#ifndef JSK_RVIZ_PLUGIN_FACING_VISUALIZER_H_
#define JSK_RVIZ_PLUGIN_FACING_VISUALIZER_H_

#include <OGRE/OgreTexture.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreManualObject.h>
#include <rviz/display_context.h>
#include <ros/time.h>
#include "overlay_utils.h"

namespace jsk_rviz_plugin
{

  // outer radius
  // inner radius
  class SquareObject
  {
  public:
    typedef boost::shared_ptr<SquareObject> Ptr;
    SquareObject(Ogre::SceneManager* manager,
                 double outer_radius,
                 double inner_radius,
                 std::string name);
    virtual ~SquareObject();
    virtual Ogre::ManualObject* getManualObject();
    virtual void setOuterRadius(double outer_radius);
    virtual void setInnerRadius(double inner_radius);
    virtual void rebuildPolygon();
  protected:
    Ogre::ManualObject* manual_;
    Ogre::SceneManager* manager_;
    double outer_radius_;
    double inner_radius_;
    std::string name_;
  private:
  };

  class TextureObject           // utility class for texture
  {
  public:
    typedef boost::shared_ptr<TextureObject> Ptr;
    TextureObject(const int width, const int height, const std::string name);
    virtual ~TextureObject();
    virtual int getWidth() { return width_; };
    virtual int getHeight() { return height_; };
    virtual ScopedPixelBuffer getBuffer();
    virtual std::string getMaterialName();
  protected:
    Ogre::TexturePtr texture_;
    Ogre::MaterialPtr material_;
    const int width_;
    const int height_;
    const std::string name_;
  private:
    
  };

  class FacingObject
  {
  public:
    typedef boost::shared_ptr<FacingObject> Ptr;
    FacingObject(Ogre::SceneManager* manager,
                 Ogre::SceneNode* parent,
                 double size);
    virtual ~FacingObject();
    virtual void setPosition(Ogre::Vector3& pos);
    virtual void setOrientation(rviz::DisplayContext* context);
    virtual void update(float wall_dt, float ros_dt) = 0;
    virtual void setSize(double size);
  protected:
    Ogre::SceneManager* scene_manager_;
    Ogre::SceneNode* node_;
    double size_;
  private:
    
  };
  
  class FacingTexturedObject: public FacingObject
  {
  public:
    typedef boost::shared_ptr<FacingTexturedObject> Ptr;
    FacingTexturedObject(Ogre::SceneManager* manager,
                         Ogre::SceneNode* parent,
                         double size);
    virtual void setSize(double size);
  protected:
    SquareObject::Ptr square_object_;
    TextureObject::Ptr texture_object_;

  private:
    
  };

  
  class GISCircleVisualizer: public FacingTexturedObject
  {
  public:
    typedef boost::shared_ptr<GISCircleVisualizer> Ptr;
    GISCircleVisualizer(Ogre::SceneManager* manager,
                        Ogre::SceneNode* parent,
                        double size,
                        std::string text = "");
    virtual void setText(std::string text) { text_ = text; }
    virtual void update(float wall_dt, float ros_dt);
    virtual void setAnonymous(bool anonymous);
  protected:
    bool anonymous_;
    std::string text_;
  private:
  };
  
}

#endif
