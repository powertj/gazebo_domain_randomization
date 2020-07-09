#include <gazebo_textures_plugin/gazebo_textures_plugin.h>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/ogre_gazebo.h>
#include <gazebo/common/Events.hh>
namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(GazeboTexturePlugin)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboTexturePlugin::GazeboTexturePlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboTexturePlugin::~GazeboTexturePlugin()
{
  //Finalize gazebo node
  this->gzNode->Fini();

  // Finalize the controller
  this->rosnode_->shutdown();
  //this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboTexturePlugin::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
{
  ROS_DEBUG_STREAM_NAMED("texture_plugin","Load");

  // save pointers
  this->_visual = _parent;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("physics_plugin", "A ROS node for Gazebo has not been initialized, unable to load plugin.");
    return;
  }

  this->rosnode_ = new ros::NodeHandle("~");

  //Start Gazebo node and create ~/visual publisher
  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();


  ros::AdvertiseServiceOptions set_material_aso =
    ros::AdvertiseServiceOptions::create<gazebo_ext_msgs::SetLinkMaterial>(
    "set_material", boost::bind(&GazeboTexturePlugin::setLinkMaterial,
    this, _1, _2), ros::VoidPtr(), &this->queue_);
  this->set_material_service_ = this->rosnode_->advertiseService(set_material_aso);

  update_connection = event::Events::ConnectPreRender(boost::bind(&GazeboTexturePlugin::Update, this));

  // start custom queue
  //this->callback_queue_thread_ =
  //  boost::thread(boost::bind(&GazeboTexturePlugin::QueueThread, this));
  ROS_INFO_NAMED("texture_plugin", "Finished loading Gazebo Texture Plugin.");
}

bool GazeboTexturePlugin::setLinkMaterial(gazebo_ext_msgs::SetLinkMaterial::Request &req,
                                          gazebo_ext_msgs::SetLinkMaterial::Response &res)
{
  boost::lock_guard<boost::mutex> lock(this->lock_);

  // Get scene pointer
  rendering::ScenePtr scene = rendering::get_scene();

  // Wait until the scene is initialized.
  if (!scene || !scene->Initialized())
  {
    res.success = false;
    res.status_message = "setLinkVisualProperties: Could not access the scene!";
    return true;
  }
  rendering::VisualPtr visual = scene->GetVisual(req.link_visual_name);
  if (!visual)
  {
    res.success = false;
    res.status_message = "setLinkVisualProperties: Could not access the visual!";
    return true;
  }

  //std::string mat = getRandomMaterial();
  //rendering::VisualPtr visual = this->_visual;
  std::string material = req.texture_name; //getRandomMaterial();
  visual->SetMaterial(material, false, false);

  res.success = true;
  return true;
}

std::string GazeboTexturePlugin::getRandomMaterial()
{
    std::string material_name;
    Ogre::ResourceManager::ResourceMapIterator resources =
        Ogre::MaterialManager::getSingleton().getResourceIterator();

    std::string s1 = "chess";
    std::string s2 = "gradient";
    std::string s3 = "flat";
    std::string s4 = "perlin";

    for (auto & material : resources)
    {
        std::string name = material.second->getName();
        if (name.find(s1) == std::string::npos)
        {
            if (name.find(s2) == std::string::npos)
            {
                if (name.find(s3) == std::string::npos)
                {
                    if (name.find(s3) == std::string::npos)
                    {
                        std::cout << name << "\n";
                    }
                }
             }
        }
    }

    return material_name;
}


////////////////////////////////////////////////////////////////////////////////
void GazeboTexturePlugin::Update()
{
  static const double timeout = 0.001;
  if (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

////////////////////////////////////////////////////////////////////////////////


void GazeboTexturePlugin::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

}

