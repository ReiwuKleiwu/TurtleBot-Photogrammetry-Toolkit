# generated custom environment hook for Gazebo GUI plugins

if [ -d "$AMENT_CURRENT_PREFIX/share/particlefilter_gz_gui_plugins/lib" ]; then
  ament_prepend_unique_value GZ_GUI_PLUGIN_PATH "$AMENT_CURRENT_PREFIX/share/particlefilter_gz_gui_plugins/lib"
fi
