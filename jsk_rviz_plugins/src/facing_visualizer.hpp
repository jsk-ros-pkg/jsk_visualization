#ifndef JSK_RVIZ_PLUGIN_FACING_VISUALIZER_H_
#define JSK_RVIZ_PLUGIN_FACING_VISUALIZER_H_

#include <OgreTexture.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTexture.h>
#include <OgreMaterial.h>
#include <OgreManualObject.h>
#include <OgreCamera.h>
#include <rviz_common/display_context.hpp>
#include <rclcpp/duration.hpp>
#include <chrono>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/movable_text.hpp>
#include "overlay_utils.hpp"

#include <memory>

namespace rviz_rendering
{
class BillboardLine;
class MovableText;
}

namespace jsk_rviz_plugins
{

  // outer radius
  // inner radius
  class SquareObject
  {
  public:
    typedef std::shared_ptr<SquareObject> Ptr;
    SquareObject(Ogre::SceneManager* manager,
                 double outer_radius,
                 double inner_radius,
                 std::string name);
    enum PolygonType
    {
      CIRCLE, SQUARE
    };
    
    virtual ~SquareObject();
    virtual Ogre::ManualObject* getManualObject();
    virtual void setOuterRadius(double outer_radius);
    virtual void setInnerRadius(double inner_radius);
    virtual void rebuildPolygon();
    virtual void setPolygonType(PolygonType type);
  protected:
    Ogre::ManualObject* manual_;
    Ogre::SceneManager* manager_;
    double outer_radius_;
    double inner_radius_;
    std::string name_;
    PolygonType polygon_type_;
  private:
  };

  class TextureObject           // utility class for texture
  {
  public:
    typedef std::shared_ptr<TextureObject> Ptr;

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
    typedef std::shared_ptr<FacingObject> Ptr;
    FacingObject(Ogre::SceneManager* manager,
                 Ogre::SceneNode* parent,
                 double size);
    virtual ~FacingObject();
    virtual void setPosition(Ogre::Vector3& pos);
    virtual void setOrientation(rviz_common::DisplayContext* context);
    virtual void setOrientation(Ogre::Quaternion& rot); // non facing API
    virtual void update(float wall_dt, float ros_dt) = 0;
    virtual void setSize(double size);
    virtual void setEnable(bool enable);
    virtual void setColor(QColor color);
    virtual void setColor(Ogre::ColourValue color);
    virtual void setText(std::string text);
    virtual void setAlpha(double alpha);
  protected:
    virtual void updateColor() = 0;
    virtual void updateText() = 0;
    Ogre::SceneManager* scene_manager_;
    Ogre::SceneNode* node_;
    Ogre::ColourValue color_;
    double size_;
    bool enable_;
    std::string text_;
  private:
    
  };

  class SimpleCircleFacingVisualizer: public FacingObject
  {
  public:
    typedef std::shared_ptr<SimpleCircleFacingVisualizer> Ptr;

    SimpleCircleFacingVisualizer(Ogre::SceneManager* manager,
                                 Ogre::SceneNode* parent,
                                 rviz_common::DisplayContext* context,
                                 double size,
                                 std::string text = "");
    virtual ~SimpleCircleFacingVisualizer();
    virtual void update(float wall_dt, float ros_dt) override;
    virtual void reset();
    
    virtual void setSize(double size);
    virtual void setEnable(bool enable);
    virtual void setText(std::string text);
  protected:
    virtual void updateArrowsObjects(Ogre::ColourValue color);
    virtual void createArrows(rviz_common::DisplayContext* context);
    virtual void updateLine();
    virtual void updateTextUnderLine();
    virtual void updateText();
    virtual void updateColor();
    rviz_rendering::BillboardLine* line_;
    rviz_rendering::BillboardLine* text_under_line_;
    Ogre::ManualObject* upper_arrow_;
    Ogre::ManualObject* lower_arrow_;
    Ogre::ManualObject* left_arrow_;
    Ogre::ManualObject* right_arrow_;
    Ogre::SceneNode* upper_arrow_node_;
    Ogre::SceneNode* lower_arrow_node_;
    Ogre::SceneNode* left_arrow_node_;
    Ogre::SceneNode* right_arrow_node_;
    Ogre::MaterialPtr upper_material_;
    Ogre::MaterialPtr lower_material_;
    Ogre::MaterialPtr left_material_;
    Ogre::MaterialPtr right_material_;
    std::string upper_material_name_;
    std::string left_material_name_;
    std::string lower_material_name_;
    std::string right_material_name_;
    rviz_rendering::MovableText* msg_;
    Ogre::SceneNode* target_text_node_;
    rclcpp::Clock clock_;
  private:
    
  };
  
  class FacingTexturedObject: public FacingObject
  {
  public:
    typedef std::shared_ptr<FacingTexturedObject> Ptr;

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
    typedef std::shared_ptr<GISCircleVisualizer> Ptr;
    GISCircleVisualizer(Ogre::SceneManager* manager,
                        Ogre::SceneNode* parent,
                        double size,
                        std::string text = "");
    virtual void setText(std::string text) { text_ = text; }
    virtual void update(float wall_dt, float ros_dt) override;
    virtual void setAnonymous(bool anonymous);
  protected:
    virtual void updateColor() {}
    virtual void updateText() {}
    virtual void setSize(double size);
    bool anonymous_;
    std::string text_;
    rclcpp::Clock clock_;
  private:
  };
  
}

#endif