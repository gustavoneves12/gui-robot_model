require 'vizkit'

view3d = Vizkit.vizkit3d_widget

#single_link_model = Vizkit.default_loader.RobotVisualization
#single_link_model.modelFile = File.join(Dir.pwd, 'test_data', 'single_link', 'model.sdf')

#multiple_visuals_model = Vizkit.default_loader.RobotVisualization
#multiple_visuals_model.modelFile = File.join(Dir.pwd, 'test_data', 'multiple_visuals', 'model.sdf')

#multiple_links_model = Vizkit.default_loader.RobotVisualization
#multiple_links_model.modelFile = File.join(Dir.pwd, 'test_data', 'multiple_links', 'model.sdf')

with_joints_model = Vizkit.default_loader.RobotVisualization
with_joints_model.modelFile = File.join(Dir.pwd, 'test_data', 'with_joints', 'model.sdf')

view3d.show
Vizkit.exec