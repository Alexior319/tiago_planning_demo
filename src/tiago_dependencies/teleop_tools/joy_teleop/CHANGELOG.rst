^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joy_teleop
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.3 (2022-04-29)
------------------
* Merge branch 'gallium-fix' into 'erbium-devel'
  Gallium fix
  See merge request ros-overlays/teleop_tools!2
* Gallium fix
* Contributors: Jordan Palacios, davidterkuile

0.3.2 (2019-08-22)
------------------
* Add missing dependency
* Contributors: Victor Lopez

0.3.1 (2019-03-28)
------------------
* Merge branch 'incrementer' into 'kinetic-devel'
  Fix head drift
  See merge request ros-overlays/teleop_tools!1
* Fix head drift
* Contributors: Adria Roig, Victor Lopez

0.3.0 (2019-01-03)
------------------
* Fill in the timestamp of outgoing messages, if applicable.
* add service example
* Add option for persistent service, defaulted false
* Contributors: AndyZe, Jeremie Deray, Bence Magyar

0.2.6 (2018-04-06)
------------------
* Support using buttons and axis in the same message
* Contributors: Tim Clephas

0.2.5 (2017-04-21)
------------------
* Remove duplicate examples, add list ones
* Contributors: Bence Magyar

0.2.4 (2016-11-30)
------------------
* Replace joy_teleop.fill_msg with genpy.message.fill_message_args
* Contributors: Stephen Street

0.2.3 (2016-07-18)
------------------
* Add hello publish to example
* Rename to fix example launch file
* Added example of feature to config file
* Added message_value parameter to specify message content on topics
* PEP8 style stuff
* Fixes bug when keep asking for increments
  would make the goal position grow infinitely instead of be of maximum 'current joint position' + 'increment quantity'
* Contributors: Bence Magyar, Sam Pfeiffer, SomeshDaga

0.2.2 (2016-03-24)
------------------
* Add install rules for example files
* gracefully handle missing joy axes
* Contributors: Bence Magyar, Kopias Peter

0.2.1 (2016-01-29)
------------------
* Add support for services
  it is now possible to asynchronously send service requests on button presses
* Adds queue_size keyword
* Contributors: Bence Magyar, Nils Berg, Enrique Fernandez

0.2.0 (2015-08-03)
------------------
* Add example for incrementer
* Update package.xmls
* Add incrementer_server
* Contributors: Bence Magyar

0.1.2 (2015-02-15)
------------------
* joy_teleop: fix minor typo
* Contributors: G.A. vd. Hoorn

0.1.1 (2014-11-17)
------------------
* Change maintainer
* checks for index out of bounds in buttons list
  `buttons` is a list, not a dict
  Filter out buttons not available
* Check for b in buttons
* Check for IndexError
* joy_teleop: add action server auto-refresh
* Move everything to joy_teleop subfolder
* Contributors: Bence Magyar, Enrique Fernández Perdomo, Paul Mathieu

0.1.0 (2013-11-28)
------------------
* joy_teleop: nice, generic joystick control for ROS
