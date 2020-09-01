# Description:
This package has two file main.cpp and database.cpp. It acts as bridge between the GUI and the detection.
It used the sqlite database for storing the data with soci interface.

# Starting this nodes:

	catkin build database
	rosrun database main

On Start it should directly establish the database connection.

# Starting the sqlite interface:

	sqlitebrowser

This will start sqlite GUI which can be used to create/edit and view the data of the tables