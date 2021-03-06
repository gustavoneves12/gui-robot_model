#ifndef robot_model_RobotVisualization_H
#define robot_model_RobotVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <base/samples/Joints.hpp>
#include <base/samples/RigidBodyState.hpp>
#include "RobotModel.h"

namespace vizkit3d
{
    class RigidBodyStateVisualization;

class RobotVisualization
        : public vizkit3d::Vizkit3DPlugin<base::samples::Joints>,
          public vizkit3d::VizPluginAddType<base::samples::RigidBodyState>,
          public RobotModel
        , boost::noncopyable
{
    Q_OBJECT
    Q_PROPERTY(QString modelFile READ modelFile WRITE setModelFile)
    Q_PROPERTY(bool framesEnabled READ areFramesEnabled WRITE setFramesEnabled)
    Q_PROPERTY(double jointsSize READ getJointsSize WRITE setJointsSize)
    Q_PROPERTY(bool followModelWithCamera READ getFollowModelWithCamera WRITE setFollowModelWithCamera)

public:
    RobotVisualization();
    ~RobotVisualization();

    void setModelFile(QString modelFile);
    QString modelFile() const;

    Q_INVOKABLE void updateData(base::samples::Joints const &sample)
    {vizkit3d::Vizkit3DPlugin<base::samples::Joints>::updateData(sample);}

    Q_INVOKABLE void updateData(base::samples::RigidBodyState const &sample)
    {vizkit3d::Vizkit3DPlugin<base::samples::Joints>::updateData(sample);}

public slots:
    bool areFramesEnabled() const;
    void setFramesEnabled(bool value);
    /** Joints Frame using RigidBodyStateVisualization
    * The default is 0.1
    */
    double getJointsSize() const;
    /** Joints Frame using RigidBodyStateVisualization
    */
    void setJointsSize(double size);

    /**
     * camera rotation center follows the robot
     */
    bool getFollowModelWithCamera() const;
    void setFollowModelWithCamera(bool value);


protected:
    virtual osg::ref_ptr<osg::Node> createMainNode();
    virtual void updateMainNode(osg::Node* node);
    virtual void updateDataIntern(base::samples::Joints const& plan);
    virtual void updateDataIntern(base::samples::RigidBodyState const& pos);

    void deleteFrameVisualizers();

private:
    struct Data;
    bool framesEnabled_;
    bool followModelWithCamera;
    double joints_size;
    Data* p;
    QString _modelFile;
    std::map<std::string, RigidBodyStateVisualization*> _frameVisualizers;

    osg::ref_ptr<osg::PositionAttitudeTransform> modelPos;
};
}
#endif
