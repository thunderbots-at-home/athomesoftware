convert: talos_base.urdf.xacro
	rosrun xacro xacro.py talos_base.urdf.xacro > talos_base.urdf
	rosparam set /robot_description -t talos_base.urdf

clean:
	rm talos_base.urdf
