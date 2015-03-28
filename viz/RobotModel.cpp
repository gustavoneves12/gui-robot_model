#include "RobotModel.h"
#include <urdf_parser/urdf_parser.h>
#include "fstream"
#include "sstream"
#include "osg/Texture2D"
#include "osg/BlendFunc"
#include "osg/AlphaFunc"
#include "osg/Billboard"
#include "osg/PointSprite"
#include "osg/Point"
#include "osg/Geometry"
#include "osg/Image"
#include "osg/Material"
#include "osg/ShapeDrawable"
#include "osg/TextureRectangle"
#include "osg/TexMat"
//#include <resource_retriever/retriever.h>
//#include "ros/ros.h"
#include "OSGHelpers.hpp"
#include <base/Logging.hpp>
#include <QFileInfo>

#include <fstream>
#include <streambuf>

#define CALL_MEMBER_POINTER_FUNCTION(obj, fnc) ((obj).*(fnc))


void addSegmentRecursive(std::map<std::string, std::vector<std::string> > map,
                        std::string parent_name,
                        KDL::Tree& tree)
{

    std::map<std::string, std::vector<std::string> >::iterator parent_itr = map.find(parent_name);

    if (parent_itr != map.end()){
        for (std::vector<std::string>::iterator child_itr = parent_itr->second.begin(); child_itr != parent_itr->second.end(); child_itr++) {

            KDL::Joint joint;
            KDL::Frame f_tip;
            KDL::RigidBodyInertia I;
            KDL::Segment segment(*child_itr, joint, f_tip, I);

            tree.addSegment(segment, parent_name);

            addSegmentRecursive(map, *child_itr, tree);
        }
    }
}

OSGSegment::OSGSegment(osg::Node* node, KDL::Segment seg)
{
    isSelected_=false;
    seg_ = seg;
    jointPos_ = 0;
    label_ = 0;
    visual_ = 0;
    toTipOsg_ = node->asTransform()->asPositionAttitudeTransform();
    updateJoint();
}

osg::ref_ptr<osg::Group> OSGSegment::getGroup() const{
    return toTipOsg_;
}

void OSGSegment::updateJoint(){
    toTipKdl_ = seg_.pose(jointPos_);
    kdl_to_osg(toTipKdl_, *toTipOsg_);
}

void OSGSegment::attachVisuals(std::vector<boost::shared_ptr<urdf::Visual> > &visual_array, QDir prefix)
{
    std::vector<boost::shared_ptr<urdf::Visual> >::iterator itr = visual_array.begin();
    std::vector<boost::shared_ptr<urdf::Visual> >::iterator itr_end = visual_array.end();

    for(itr = visual_array.begin(); itr != itr_end; ++itr)
    {
        boost::shared_ptr<urdf::Visual> visual = *itr;
        attachVisual(visual, prefix);
    }
}

void OSGSegment::attachVisual(boost::shared_ptr<urdf::Visual> visual, QDir baseDir)
{
    osg::PositionAttitudeTransform* to_visual = new osg::PositionAttitudeTransform();
    if (visual)
        urdf_to_osg(visual->origin, *to_visual);
    toTipOsg_->addChild(to_visual);
    toTipOsg_->setName(seg_.getJoint().getName());

    osg::Node* osg_visual = 0;
    if(visual && visual->geometry->type == urdf::Geometry::MESH){
        urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(visual->geometry.get());
        to_visual->setScale(urdf_to_osg(mesh->scale));

        std::string prefix = "file://";
        std::string filename = "";
        if(mesh->filename.compare(0, prefix.length(), prefix) == 0){
            filename = mesh->filename.substr(prefix.length());
        }
        else
            filename = mesh->filename;
        LOG_DEBUG("Trying to load mesh file %s", filename.c_str());

        QString qfilename = QString::fromStdString(filename);
        if (QFileInfo(qfilename).isRelative())
            filename = baseDir.absoluteFilePath(qfilename).toStdString();

        osg_visual = osgDB::readNodeFile(filename);
        if(!osg_visual){
            LOG_ERROR("OpenSceneGraph did not succees loading the mesh file %s.", filename.c_str());
            throw std::runtime_error("Error loading mesh file.");
        }
        if(!osg_visual){
            LOG_ERROR("Unecpected error loading mesh file %s", mesh->filename.c_str());
            throw(std::runtime_error("Couldn't load mesh file."));
        }
    }
    else if(visual && visual->geometry->type == urdf::Geometry::BOX){
        urdf::Box* box = dynamic_cast<urdf::Box*>(visual->geometry.get());
        osg::ShapeDrawable* drawable = new osg::ShapeDrawable(new osg::Box(osg::Vec3d(0,0,0), box->dim.x, box->dim.y, box->dim.z));
        osg_visual = new osg::Geode;
        osg_visual->asGeode()->addDrawable(drawable);
    }
    else if(visual && visual->geometry->type == urdf::Geometry::CYLINDER){
        urdf::Cylinder* cylinder = dynamic_cast<urdf::Cylinder*>(visual->geometry.get());
        osg::ShapeDrawable* drawable = new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3d(0,0,0), cylinder->radius, cylinder->length));

        osg_visual = new osg::Geode;
        osg_visual->asGeode()->addDrawable(drawable);
    }
    else if(visual && visual->geometry->type == urdf::Geometry::SPHERE){
        urdf::Sphere* sphere = dynamic_cast<urdf::Sphere*>(visual->geometry.get());
         osg::ShapeDrawable* drawable = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3d(0,0,0), sphere->radius));

        osg_visual = new osg::Geode;
        osg_visual->asGeode()->addDrawable(drawable);
    }
    else
    {
        osg_visual = new osg::Geode;
    }

    //Set material
    if(visual){
        if(visual->material){
            osg::ref_ptr<osg::StateSet> nodess = osg_visual->getOrCreateStateSet();
            nodess->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

            osg::ref_ptr<osg::Material> nodematerial = new osg::Material;

            std::string filename = visual->material->texture_filename;
            if(filename != ""){
                QString qfilename = QString::fromStdString(visual->material->texture_filename);
                if (QFileInfo(qfilename).isRelative())
                    filename = baseDir.absoluteFilePath(qfilename).toStdString();
                osg::ref_ptr<osg::Image> texture_img = osgDB::readImageFile(filename);
                if(!texture_img){
                    std::stringstream ss;
                    ss << "Could not load texture from file '"<<visual->material->texture_filename<<"'.";
                    throw(std::runtime_error(ss.str()));
                }
                osg::ref_ptr<osg::TextureRectangle> texture_rect = osg::ref_ptr<osg::TextureRectangle>(new osg::TextureRectangle(texture_img));
                osg::ref_ptr<osg::TexMat> texmat = new osg::TexMat;
                texmat->setScaleByTextureRectangleSize(true);
                nodess->setTextureAttributeAndModes(0, texture_rect, osg::StateAttribute::ON);
                nodess->setTextureAttributeAndModes(0, texmat, osg::StateAttribute::ON);
            }
            //Specifying the colour of the object
            nodematerial->setDiffuse(osg::Material::FRONT,osg::Vec4(visual->material->color.r,
                                                                    visual->material->color.g,
                                                                    visual->material->color.b,
                                                                    visual->material->color.a));
            nodematerial->setSpecular(osg::Material::FRONT,osg::Vec4(0.2,
                                                                     0.2,
                                                                     0.2,
                                                                     1));

            //Attaching the newly defined state set object to the node state set
            nodess->setAttribute(nodematerial.get());
        }
    }

    to_visual->addChild(osg_visual);
    to_visual->setName(seg_.getName());
    to_visual->setUserData(this);
    visual_ = osg_visual->asGeode();
}

void OSGSegment::attachVisual(sdf::ElementPtr sdf_visual, QDir baseDir){

    osg::PositionAttitudeTransform* to_visual = new osg::PositionAttitudeTransform();
    sdf_pose_to_osg(sdf_visual->GetElement("pose"), *to_visual);

    toTipOsg_->addChild(to_visual);
    toTipOsg_->setName(seg_.getJoint().getName());

    osg::Node* osg_visual = 0;
    if (sdf_visual->HasElement("geometry")){
        sdf::ElementPtr sdf_geometry  = sdf_visual->GetElement("geometry");
        sdf::ElementPtr sdf_geom_elem = sdf_geometry->GetFirstElement();

        if (sdf_geom_elem->GetName() == "box"){
            osg::Vec3f size;
            sdf_size_to_osg(sdf_geom_elem->GetElement("size"), size);
            osg::ShapeDrawable* drawable = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0,0,0), size.x(), size.y(), size.z()));
            osg_visual = new osg::Geode;
            osg_visual->asGeode()->addDrawable(drawable);
        }
        else if (sdf_geom_elem->GetName() == "cylinder"){
            double radius = sdf_geom_elem->GetElement("radius")->Get<double>();
            double length = sdf_geom_elem->GetElement("length")->Get<double>();
            osg::ShapeDrawable* drawable = new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3d(0,0,0), radius, length));
            osg_visual = new osg::Geode;
            osg_visual->asGeode()->addDrawable(drawable);
        }
        else if (sdf_geom_elem->GetName() == "sphere"){
            double radius = sdf_geom_elem->GetElement("radius")->Get<double>();
            osg::ShapeDrawable* drawable = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3d(0,0,0), radius));
            osg_visual = new osg::Geode;
            osg_visual->asGeode()->addDrawable(drawable);
        }
        else if  (sdf_geom_elem->GetName() == "mesh"){
            osg::Vec3 scale;
            sdf_scale_to_osg(sdf_geom_elem->GetElement("scale"), scale);

            to_visual->setScale(scale);

            std::string uri = sdf_geom_elem->GetElement("uri")->Get<std::string>();

            std::string model_prefix = "model://";

            std::string filename = "";

            if(uri.compare(0, model_prefix.length(), model_prefix) == 0){
                filename = uri.substr(model_prefix.length());
            }
            else {
                filename = uri;
            }

            QString qfilename = QString::fromStdString(filename);
            if (QFileInfo(qfilename).isRelative()){
//                filename = baseDir.absoluteFilePath(QFileInfo(qfilename).fileName()).toStdString();
                filename = baseDir.absoluteFilePath(QFileInfo(qfilename).baseName()).toStdString();
                filename += ".osg";
            }

            std::cout << "filename: " << filename << std::endl;

            osg_visual = osgDB::readNodeFile(filename);

            if (!osg_visual) {
                LOG_ERROR("OpenSceneGraph did not succees loading the mesh file %s.", filename.c_str());
                throw std::runtime_error("Error loading mesh file.");
            }

        }
        else {
            osg_visual = new osg::Geode;
        }

    }

    to_visual->addChild(osg_visual);
    to_visual->setName(seg_.getName());
    to_visual->setUserData(this);

    visual_ = osg_visual->asGeode();
}

void OSGSegment::attachVisuals(std::vector<sdf::ElementPtr> &visual_array, QDir prefix){

    std::vector<sdf::ElementPtr>::iterator itr = visual_array.begin();
    std::vector<sdf::ElementPtr >::iterator itr_end = visual_array.end();

    for(itr = visual_array.begin(); itr != itr_end; ++itr)
    {
        sdf::ElementPtr visual = *itr;
        attachVisual(visual, prefix);
    }
}

void OSGSegment::removeLabel(){
    if(label_)
        toTipOsg_->removeChild(label_);
    label_ = 0;
}

void OSGSegment::attachLabel(std::string name, std::string filepath){
    if(label_)
        removeLabel();

    osg::Geode *geode = new osg::Geode();
    osg::Geometry *geometry = new osg::Geometry();

    osg::Vec3Array* vertices = new osg::Vec3Array;
    vertices->push_back (osg::Vec3 (0, 0, 0.0));

    geometry->setVertexArray (vertices);

    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertices->size()));

    geode->addDrawable(geometry);
    osg::StateSet *set = new osg::StateSet();

    /// Setup the point sprites
    osg::PointSprite *sprite = new osg::PointSprite();
    set->setTextureAttributeAndModes(0, sprite, osg::StateAttribute::ON);

    /// Give some size to the points to be able to see the sprite
    osg::Point *point = new osg::Point();
    point->setSize(50);
    set->setAttribute(point);

    /// Disable depth test to avoid sort problems and Lighting
    set->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    set->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    osg::ref_ptr<osg::BlendFunc> texture_blending_function = new osg::BlendFunc();
    set->setAttributeAndModes(texture_blending_function.get(), osg::StateAttribute::ON);

    osg::ref_ptr<osg::AlphaFunc> alpha_transparency_function = new osg::AlphaFunc();
    alpha_transparency_function->setFunction(osg::AlphaFunc::GEQUAL, 0.05);
    set->setAttributeAndModes(alpha_transparency_function.get(), osg::StateAttribute::ON );

    /// The texture for the sprites
    osg::Texture2D *tex = new osg::Texture2D();
    osg::Image* image = osgDB::readImageFile(filepath);
    image->flipVertical();
    tex->setImage(image);

    set->setTextureAttributeAndModes(0, tex, osg::StateAttribute::ON);

    toTipOsg_->addChild(geode);

    geode->setStateSet(set);
    geode->setName(name);
    geode->setUserData(this);

    label_ = geode;
}

bool OSGSegment::toggleSelected(){
    isSelected_ = !isSelected_;

    osg::Group* parent = visual_->getParent(0);

    if(isSelected_){
        osgFX::Outline* scribe = new osgFX::Outline();
        scribe->setWidth(1);
        scribe->setColor(osg::Vec4(1,0,0,1));
        scribe->addChild(visual_);
        parent->replaceChild(visual_,scribe);
    }
    else{
        //node already picked so we want to remove marker to unpick it.
        osg::Node::ParentList parentList = parent->getParents();
        for(osg::Node::ParentList::iterator itr=parentList.begin();
            itr!=parentList.end();
            ++itr)
        {
            (*itr)->replaceChild(parent, visual_);
        }
    }

    //update_visual();
    return isSelected_;
}


RobotModel::RobotModel(){
    //Root is the entry point to the scene graph
    osg::Group* root = new osg::Group();
    root_ = root;
    loadEmptyScene();

    loadFunctions["urdf"] = &RobotModel::loadURDF;
    loadFunctions["sdf"] = &RobotModel::loadSDF;
}

osg::Node* RobotModel::loadEmptyScene(){
    root_->removeChildren(0, root_->getNumChildren());
    jointNames_.clear();
    segmentNames_.clear();
    return root_;
}

osg::Node* RobotModel::makeOsg2(KDL::Segment kdl_seg, urdf::Link urdf_link, osg::Group* root){
    osg::PositionAttitudeTransform* joint_forward = new osg::PositionAttitudeTransform();
    OSGSegment* seg = new OSGSegment(joint_forward, kdl_seg);

    joint_forward->setUserData( seg );
    joint_forward->setUpdateCallback(new OSGSegmentCallback);

    root->addChild(joint_forward);

    //Attach one visual to joint
    if (urdf_link.visual_array.size() == 0)
    {
        boost::shared_ptr<urdf::Visual> visual = urdf_link.visual;
        seg->attachVisual(visual, rootPrefix);
    }
    //Attach several visuals to joint
    else
    {
         std::vector<boost::shared_ptr<urdf::Visual> > visual_array = urdf_link.visual_array;
         seg->attachVisuals(visual_array, rootPrefix);
    }

    return joint_forward;
}

osg::Node* RobotModel::makeOsg2(KDL::Segment kdl_seg, sdf::ElementPtr sdf_link, osg::Group* root){

    std::vector<sdf::ElementPtr> visuals;

    if (sdf_link->HasElement("visual")){

        sdf::ElementPtr visualElem= sdf_link->GetElement("visual");
        while (visualElem){
            visuals.push_back(visualElem);
            visualElem = visualElem->GetNextElement("visual");
        }
    }


    osg::PositionAttitudeTransform* joint_forward = new osg::PositionAttitudeTransform();
    OSGSegment* seg = new OSGSegment(joint_forward, kdl_seg);

    joint_forward->setUserData( seg );
    joint_forward->setUpdateCallback(new OSGSegmentCallback);

    root->addChild(joint_forward);

    if (visuals.size() > 0)
    {
        seg->attachVisuals(visuals, rootPrefix);
    }
    else {
        seg->attachVisual(visuals[0], rootPrefix);
    }

    return joint_forward;
}

osg::Node* RobotModel::makeOsg( boost::shared_ptr<urdf::ModelInterface> urdf_model ){
    //Parse also to KDL
    KDL::Tree tree;
    kdl_parser::treeFromUrdfModel(*urdf_model, tree);

    //
    // Here we perform a full traversal throu the kinematic tree
    // hereby we go depth first
    //
    boost::shared_ptr<const urdf::Link> urdf_link; //Temp Storage for current urdf link
    KDL::Segment kdl_segment; //Temp Storage for urrent KDL link (same as URDF, but already parsed to KDL)
    osg::Node* hook = 0; //Node (from previous segment) to hook up next segment to

    std::vector<boost::shared_ptr<const urdf::Link> > link_buffer; //Buffer for links we still need to visit
    //used after LIFO principle
    std::vector<osg::Node*> hook_buffer;                  //Same as above but for hook. The top most
    //element here corresponds to the hook of the
    //previous depth level in the tree.
    link_buffer.push_back(urdf_model->getRoot()); //Initialize buffers with root
    hook_buffer.push_back(root_);
    while(!link_buffer.empty()){
        //get current node in buffer
        urdf_link = link_buffer.back();
        link_buffer.pop_back();

        //FIXME: This is hacky solution to prevent from links being added twice. There should be a better one
        if(std::find (segmentNames_.begin(), segmentNames_.end(), urdf_link->name) != segmentNames_.end())
            continue;

        //expand node
        link_buffer.reserve(link_buffer.size() + std::distance(urdf_link->child_links.begin(), urdf_link->child_links.end()));
        link_buffer.insert(link_buffer.end(), urdf_link->child_links.begin(), urdf_link->child_links.end());

        //create osg link
        hook = hook_buffer.back();
        hook_buffer.pop_back();
        kdl_segment = tree.getSegment(urdf_link->name)->second.segment;
        osg::Node* new_hook = makeOsg2(kdl_segment,
                                       *urdf_link, hook->asGroup());

        //Also store names of links and joints
        segmentNames_.push_back(kdl_segment.getName());
        if(kdl_segment.getJoint().getType() != KDL::Joint::None)
            jointNames_.push_back(kdl_segment.getJoint().getName());

        //Append hooks
        for(uint i=0; i<urdf_link->child_links.size(); i++)
            hook_buffer.push_back(new_hook);
    }
    return root_;
}

osg::Node* RobotModel::makeOsg( sdf::ElementPtr sdf_model )
{

    std::string model_name = sdf_model->GetAttribute("name")->GetAsString();

    std::map<std::string, sdf::ElementPtr> child_link_joint_map;

    if (sdf_model->HasElement("joint")){
        sdf::ElementPtr jointElem = sdf_model->GetElement("joint");

        while (jointElem){
            std::string child_link_name = jointElem->GetElement("child")->Get<std::string>();
            child_link_joint_map.insert(std::make_pair<std::string, sdf::ElementPtr>(child_link_name, jointElem));
            jointElem = jointElem->GetNextElement("joint");
        }
    }


    std::map<std::string, sdf::ElementPtr> links_map;

    if (sdf_model->HasElement("link")){
        sdf::ElementPtr linkElem = sdf_model->GetElement("link");
        while (linkElem){
            std::string link_name = linkElem->Get<std::string>("name");
            links_map.insert(std::make_pair<std::string, sdf::ElementPtr>(link_name, linkElem));
            linkElem = linkElem->GetNextElement("link");
        }
    }

    std::map<std::string, sdf::ElementPtr>::iterator links_it;
    std::map<std::string, sdf::ElementPtr>::iterator child_link_joint_it;
    std::map<std::string, std::vector<std::string> > parent_childs_map;

    for (links_it = links_map.begin(); links_it != links_map.end(); links_it++) {
        std::string child_name = links_it->first;
        std::string parent_name = "root";

        if ((child_link_joint_it = child_link_joint_map.find(child_name)) != child_link_joint_map.end()) {
            parent_name = child_link_joint_it->second->GetElement("parent")->Get<std::string>();
        }

        std::cout << "parent_name: " << parent_name << std::endl;
        std::cout << "child_name: " << child_name << std::endl;

        std::map<std::string, std::vector<std::string> >::iterator parent_it = parent_childs_map.find(parent_name);

        if (parent_it == parent_childs_map.end()){
            std::vector<std::string> childs;
            childs.push_back(child_name);
            parent_childs_map.insert(std::make_pair<std::string, std::vector<std::string> >(parent_name, childs));
        }
        else {
            parent_it->second.push_back(child_name);
        }
    }

    KDL::Tree tree("root");
    addSegmentRecursive(parent_childs_map, "root", tree);


    KDL::SegmentMap::const_iterator root = tree.getRootSegment();

    std::vector<KDL::SegmentMap::const_iterator> segment_buffer;
    std::vector<osg::Node*> hook_buffer;

    osg::Node* hook = 0;
    segment_buffer.reserve(segment_buffer.size() + std::distance(root->second.children.begin(), root->second.children.end()));
    segment_buffer.insert(segment_buffer.end(), root->second.children.begin(), root->second.children.end());

    for(uint i = 0; i < root->second.children.size(); i++){
        hook_buffer.push_back(root_);
    }


    while (!segment_buffer.empty()){

        std::map<std::string, sdf::ElementPtr>::const_iterator link_element;

        KDL::SegmentMap::const_iterator it = segment_buffer.back();
        segment_buffer.pop_back();
        KDL::Segment kdl_segment = it->second.segment;

        if (it->second.children.size() > 0){
            segment_buffer.reserve(segment_buffer.size() + std::distance(it->second.children.begin(), it->second.children.end()));
            segment_buffer.insert(segment_buffer.end(), it->second.children.begin(), it->second.children.end());
        }

        if (links_map.find(kdl_segment.getName()) != links_map.end()){
            sdf::ElementPtr sdf_link = links_map[kdl_segment.getName()];

            //create osg link
            hook = hook_buffer.back();
            hook_buffer.pop_back();

            osg::Node* new_hook = 0;
            new_hook = makeOsg2(kdl_segment, sdf_link, hook->asGroup());

            //Append hooks
            for(uint i = 0; i < it->second.children.size(); i++){
                hook_buffer.push_back(new_hook);
            }

        }

    }

    return root_;
}

osg::Node* RobotModel::load(QString path){

    loadEmptyScene();

    QString suffix = QFileInfo(path).suffix().toLower();

    if (loadFunctions.contains(suffix)){
        return CALL_MEMBER_POINTER_FUNCTION(*this, loadFunctions[suffix])(path);
    }
    else {
        LOG_ERROR("the %s type of file is not supported .", suffix.toStdString().c_str());
    }

    return new osg::Group();
}

osg::Node* RobotModel::loadURDF(QString path)
{
    std::ifstream t( path.toStdString().c_str() );
    std::string xml_str((std::istreambuf_iterator<char>(t)),
                       std::istreambuf_iterator<char>());

    rootPrefix = QDir(QFileInfo(path).absoluteDir());

    //Parse urdf
    boost::shared_ptr<urdf::ModelInterface> model = urdf::parseURDF( xml_str );
    if (!model)
        return NULL;

    modelName =  model->getName();

    return makeOsg(model);
}

osg::Node* RobotModel::loadSDF(QString path)
{
    rootPrefix = QDir(QFileInfo(path).absoluteDir());

    std::cout << "rootPrefix: " << rootPrefix.absolutePath().toStdString() << std::endl;

    sdf::SDFPtr sdf(new sdf::SDF);

    if (!sdf::init(sdf)){
        LOG_ERROR("unable to initialize sdf.");
        return NULL;
    }

    if (!sdf::readFile(path.toStdString(), sdf)){
        LOG_ERROR("unabled to read sdf file %s.", path.toStdString().c_str());
        return NULL;
    }

    if (!sdf->root->HasElement("model")){
        LOG_ERROR("the <model> tag not exists");
        return NULL;
    }

    return makeOsg(sdf->root->GetElement("model"));
}

OSGSegment* RobotModel::getSegment(std::string name)
{
    osg::Node* node = findNamedNode(name, root_);
    if(!node)
        return 0;

    OSGSegment* jnt = dynamic_cast<OSGSegment*>(node->getUserData());
    if(!jnt)
        return 0;

    return jnt;
}

bool RobotModel::setJointState(std::string jointName, double jointVal)
{
    osg::Node* node = findNamedNode(jointName, root_);
    if(!node)
        return false;

    OSGSegment* jnt = dynamic_cast<OSGSegment*>(node->getUserData());
    jnt->setJointPos(jointVal);
    return true;
}

bool RobotModel::setJointState(const std::map<std::string, double>& jointVals)
{
    for (std::map<std::string, double>::const_iterator it=jointVals.begin();
         it!=jointVals.end(); ++it){
        osg::Node* node = findNamedNode( it->first, root_);
        if(!node)
            return false;

        OSGSegment* jnt = dynamic_cast<OSGSegment*>(node->getUserData());
        jnt->setJointPos(it->second);
    }
    return true;
}

bool RobotModel::toggleHighlight(std::string name)
{
    OSGSegment* seg = getSegment(name);
    if(!seg)
        return false;
    seg->toggleSelected();
    return true;
}
