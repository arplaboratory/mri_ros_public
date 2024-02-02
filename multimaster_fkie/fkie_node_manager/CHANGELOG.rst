^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fkie_node_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.2 (2022-07-17)
------------------

1.3.1 (2022-07-15)
------------------

1.3.0 (2022-07-15)
------------------
* fkie_node_manager: fixed deselect nodes while start nodes on other host
* fkie_node_manager: show kill button for nodes without valid PID, too
* fkie_node_manager: editor: do not close editor on ESC
* fkie_node_manager: fixed bug with changed master selection after reload launch files
* fkie_node_manager: added output for detected ROS Master URI
* fkie_node_manager: fix restart system nodes after set time and answer 'no'
* fkie_node_manager: fix restart system nodes after set time and answer 'no'
* fkie_node_manager: fix restart system nodes after set time
* Use more conservative terminal emulator parameters
  This should make the x-terminal-emulator call more compatible with
  different types of terminals
* fkie_node_manager: remove binaries in src path if multiple executables are found
* fkie_multimaster_fkie: added python3 dependencies
* fkie_node_manager: filter binaries from src directory if more then one executable found
* fkie_node_manager: use find_resource insteadof find_node to get executable nodes
* fkie_node_manager: fixed remote start using reduced_nm module
* fkie_node_manager: fixed a bug while comment/uncomment of last line
  added Switch comment to context menu
* fkie_node_manager: fixed save gui setting on exit if stop master was selected
* fkie_node_manager: restart system nodes after set time
* fkie_node_manager: added option 'start daemon' to start dialog
* fkie_node_manager: fix for visualization of nodes with and without capability groups
* fkie_node_manager: retain selected node after reload a configuration
* fkie_node_manager: fixed missed text in editor after reload changed launch file
* fkie_node_manager: requests for name resolution
* fkie_node_manager: changed exception handling in name resolution
* check for pid before execute
* fkie_node_manager: fix kill on stop every node
* fkie_node_manager: go to search result on item selection
* fkie_node_manager: fix error on stop nodes
* fkie_node_manager: force kill if kill_on_stop is defined
* fkie_node_manager: fixed not compatible python2 code
* fkie_node_manager: fixed error in noetic, see issue `#160 <https://github.com/fkie/multimaster_fkie/issues/160>`_
* fkie_node_manager: fixed view of synced nodes
* fkie_node_manager: added links for roslog and screen log in the node description
* fkie_node_manager: fixed stop all nodes on exit
  removed warning about connection refused after shutdown call on the
  XMLRPC API of the node
* fkie_node_manager: echo dialog: added message file checkbox for quick access
* fkie_node_manager: changed requests for diagnostic messages to avoid multiple messages if sync is enabled
* fkie_node_manager: fix for visualization in environments with ROS_IP
* fkie_node_manager: fixed crash on daemon exception
* fkie_node_manager: added throttle daemon error output
* fkie_node_manager: added info to a host if no quality is available
* fkie_node_manager: fixed threaded call to get remote user
* fkie_node_manager: do not block on username request
* fkie_node_manager: add nodes to view with not reachable nodeuri
* fkie_node_manager: fixed resize of control buttons
* fkie_node_manager: changed terminal detection
* fkie_node_manager: fix remote log view
* fkie_node_manager: increased the update rate for analysis of diagnostic messages
* fkie_node_manager: set text selectable in parameter dialog
* fkie_node_manager: renamed parameter
  Open screen on activate -> Use internal log widget
* fkie_node_manager: changed question behaviour on changed binaries
* fkie_node_manager: fixed 'Open screen on activate' option
* fkie_node_manager: open log file in new terminal or dock widget depending on new parameter
  new parameter: Open screen on activate
* added 'Open screen on activate' parameter
* fkie_node_manager: stop/restart only filtered nodes
* fkie_node_manager: changed state visualization of running nodes
* fkie_node_manager: resize control buttons
* fkie_node_manager: fixed propagation of node states to parent groups
* fkie_node_manager: fixed visualization of remote nodes connected to local master using ROS_MASTER_URI
* fkie_node_manager: reduce control buttons on size change
* fkie_node_manager: fixed crash after change of configuration
* added detection if master and daemon a launched with different user
* changed handling of the diagnostics level in group view
* fixed remote start of roscore
* added label for host name in launch widget
* added host highlighting in launch editor
* deselect all log level after change
* sort logger level
* fixed change all loglevel
* fkie_node_manager: update logger before they are available
* Contributors: Alexander Tiderko, Timo Röhling

1.2.7 (2021-01-23)
------------------

1.2.6 (2021-01-16)
------------------
* replace escape sequences in service responses
* Contributors: Alexander Tiderko

1.2.5 (2021-01-16)
------------------

1.2.4 (2020-11-11)
------------------
* renamed 'associations' and 'kill_on_stop' parameter and add dapricated notifications
  new names: nm/associations nm/kill_on_stop
* fkie_node_manager: changed from /rosout to /rosout_agg
  a lot of nodes lead to alot of topic connection and slow gui
* fkie_node_manager: fix double start of nodes while using associations
* fkie_node_manager: fixed script runner; stop also node if script dies
* fkie_node_manager: do not edit parameter with size > 32000
* fkie_node_manager: added missing webkit dependecy
* fkie_node_manager: add warning if no log file was found on local host
* fkie_node_manager: added more output to remote script
* fkie_node_manager: added copy-link to description to copy topic, service, node names to clipboard
* fkie_node_manager: changed host comparison; added search for further log files
* fkie_node_manager: show ROS log from lates folder if no one is available
* fkie_node_manager: fix start daemon if ROS_IOP is set
* fkie_node_manager: do not pull offline hosts
* fix block while name resolution
* fix for issue `#138 <https://github.com/fkie/multimaster_fkie/issues/138>`_
* fkie_node_manager: editor: clear search results on activate and search for a node
* fkie_node_manager: restore editor dialog if it already open and minimized
* fkie_node_manager: restore editor dialog if it already open and minimized
* updated diagnostic message for warnings in master_sync
* fkie_node_manager: fixed detection of included files in 'value' tags
* added logging setion to local manual
* fixed close ssh sessions
* fixed screenlog via ssh
* added chapter about associations parameter to local help
* fkie_node_manager: logscreen: improved highlighting speed
* Contributors: Alexander Tiderko, Robot User

1.2.1 (2020-07-22)
------------------
* fkie_multimaster: added conditions for python3 dependencies in package xml
* Contributors: Alexander Tiderko

1.2.0 (2020-07-22)
------------------
* fkie_node_manager_daemon: fixed rostest
* fkie_node_manager: editor: fixed recursive search
* fixed catkin_lint warnings
* fkie_node_manager: updated description for shortcuts
* fkie_node_manager: fixed open terminal
* fkie_node_manager: added troubleshooting to internal help
* fkie_node_manager: ask user if more then one binary in src
* fkie_node_manager: avoid ask for binary to select if located in devel and src
* fkie_node_manager: changed dependency from Crypto to pycryptodome
* fkie_node_manager: logscreen: show ROS log by {Ctrl,Shift}+Double Click
* fkie_node_manager: fixed join network from network discovery dialog
* fkie_node_manager: start master_sync after master_discovery if both are started using start-dialog
* fkie_node_manager: fixed python3 compatibility in logscreens
* fkie_multimaster: fixed warning for cmake_minimum_required
* fkie_node_manager: fixed create new file in launch widget
* fkie_multimaster: fixed build/start in noetic
* fkie_node_manager: refactored progress queue, use now kwargs instead of args
* fkie_node_manager: use Cryptodome or Crypto depending on availability
* fkie_node_manager: show package name in node-info instead of full path
* fkie_node_manager: reorganized description of nodes, topics and services
* fkie_node_manager: added logger filter to logwidget
* fkie_node_manager: store all loggers states in logscreen while runtime
* fkie_node_manager: start assosiated nodes first
* fkie_node_manager: delete diagnostic message of a node on stop/start
* fkie_node_manager: changed log_widget to get/set log_level depending on masteruri
* fkie_node_manager: fixed short distance for drag and drop
* Contributors: Alexander Tiderko

1.1.0 (2020-05-13)
------------------
* fkie_node_manager: added detection for restarted nodes and update loglevel for open log screens
* fkie_node_manager: log screen: added rules to colorize logs without terminal escape characters
* fkie_node_manager: set loglevel after restart of node while screen widget is open
* fkie_node_manager: open ROSLOG if no screen is available
* fkie_node_manager: fixed fast hide of screen info
* fkie_node_manager: fixed compatibility of echo dialog to python 3
* fkie_node_manger: prepend stop while start master_discovery from node_manager
  it is a workaround because all nodes with same are stopped by roscore.
  Also the new one.
* prepared conditions for python3  in package xml
* fkie_multimaster_msgs: changed timestamp in MasterState from float to time
* fkie_node_manager: editor: if found multiple node with same name show both
* fkie_multimaster_fkie: removed unused messages and services
* fkie_node_manager: on multiple screen for a node open it in terminal instead of log widget
* fkie_multimaster_fkie: got get_local_address from rosgraph.network
* fkie_multimaster: a lot of merges for python 3 compatibility
* fkie_node_manager: removed dependency from GUI.qrc
* fkie_node_manager: install doc directory
* fkie_node_manager: change all loglevel in a new thread
* fkie_node_manager: added posibility to open in mulitple screen docks
* fkie_node_manager: added grep functionality to screen logger view
* fkie_node_manager: logscreen: changed highlighting and increased performance
* fkie_node_manager: added associations to custom dialog in editor
* fkie_node_manager: added associations parameter
* fkie_node_manager: update logger on view
* fkie_node_manager: select screen log tab on activate
* fkie_node_manager: added tabbed behaviour
* fkie_node_manager: added a screen log widget
  alternative view of current screen output with only window
* Contributors: Alexander Tiderko

1.0.0 (2019-04-30)
------------------
* added daemon for node manager. The daemon replaces the default_config package and adds support for remote access through gRPC.
* renamed all packages to fkie_*
* old version are availabe on branch 'old_master'

0.8.12 (2019-04-30)
-------------------
* fixed lost nodes while grouping
* Contributors: Alexander Tiderko

0.8.11 (2019-02-27)
-------------------

0.8.10 (2019-02-26)
-------------------
* node_manager_fkie: exapand (nodes, topics, services) on filter
* fixed build node_manager_fkie without .git repository issue `#91 <https://github.com/fkie/multimaster_fkie/issues/91>`_
* node_manager_fkie: fixed crash on show critical message dialog
* Contributors: Alexander Tiderko

0.8.9 (2018-12-21)
------------------
* fix install build
* Contributors: Alexander Tiderko

0.8.8 (2018-12-19)
------------------
* fixed install node_manager_fkie
* Contributors: Alexander Tiderko

0.8.7 (2018-12-18)
------------------
* node_manager_fkie: added version detection
* Contributors: Alexander Tiderko

0.8.5 (2018-12-11)
------------------
* node_manager_fkie: removed install author warning
* node_manager_fkie: fixed navigation in topic and service view
  do not open echo/call dialog on activate namespace group
* Contributors: Alexander Tiderko

0.8.4 (2018-12-08)
------------------

0.8.3 (2018-12-07)
------------------
* node_manager_fkie: added: Augment CMake script to install node_manager launcher on Ubuntu. pull request `#82 <https://github.com/fkie/multimaster_fkie/issues/82>`_ from acschaefer/master
* node_manager_fkie: added parameter to disable namespace groups
* node_manager_fkie: editor: improved seletion of node definition by moving selected text to top
* node_manager_fkie: new: apply enhancement to organize nodes view by namespaces, see issue `#83 <https://github.com/fkie/multimaster_fkie/issues/83>`_
* node_manager_fkie: fixed copy paste error
* node_manager_fkie: changed highlighting for groups and nodes
* node_manager_fkie: editor: fixed uncomment of -- statements
* node_manager_fkie: added launch file to test namespace grouping.
* node_manager_fkie: fix namespace view
* node_manager_fkie: fixed topic publish dialog for messages with arrays
* node_manager_fkie: fix crash while start master_discovery with master_sync on
* node_manager_fkie: fixed add new parameter in parameter dialog
* node_manager_fkie: added parameter for timeout to close closing dialog
* Contributors: Alexander Schaefer, Alexander Tiderko

0.8.2 (2018-08-10)
------------------
* fixed issue `#79 <https://github.com/fkie/fkie_multimaster/issues/79>`_
* Contributors: Alexander Tiderko

0.8.1 (2018-08-03)
------------------
* fkie_node_manager: changed behaviour on question to reload files and display noscreen errors
* Contributors: Alexander Tiderko

0.8.0 (2018-07-16)
------------------
* fkie_node_manager: added warning if while remote start no executable was found
  rosrun throws no error if no executable was found it is only an output.
* fkie_node_manager: fixed activation of minimized launch editor
* fkie_node_manager: added settings parameter 'movable dock widgets' to prevent dock widgets from moving
* fkie_node_manager: fixed error in select_dialog on close node_manager
* fkie_node_manager: added group icon with count of nodes inside
* fkie_node_manager: added info icons for groups
* fkie_node_manager: added timer to close exit dialog on close node_manager
* fkie_node_manager: fixed delay open io screen
* fkie_node_manager: use priority queue for sreen io only if normal queue has more than 5 elements
* fkie_node_manager: reduced update count
* fkie_node_manager: changed color of question box
* fkie_node_manager: added link for nodelet manager in description of nodelets
* fkie_node_manager: add an option to disable the question dialog while restart nodelets
* fkie_node_manager: changed background of question dialog to non transparent
* fkie_node_manager: changed question dialog for launch and transfer files
* changed visualization for available configurations, added visualisation for nodelets
  changed qestion dialog on changes of launch files and restart of
  nodelets
* fkie_node_manager: fixed trasfer of wrong files on change to remote hosts
* fkie_node_manager: editor: fix recursive search
* fkie_node_manager: fixed crash on call of an unknown service
* fkie_node_manager: fix administratively prohibited error while delete logs
  This error occurs while delete more than 10 logs on remote host
* fkie_node_manager: resolve pkg:// in all arguments
* fkie_node_manager: fix crash while assigne color
* Contributors: Alexander Tiderko

0.7.8 (2018-03-24)
------------------
* Fix catkin_lint warnings
* fkie_node_manager: fixed crash on errors while open network discovery dialog
* fkie_node_manager: fixed copy function in launch file browser
* fkie_node_manager: fixed file name copy crash
* fkie_node_manager: added more checks while handle nodelet restarts
* fkie_node_manager: added check for restart of nodelet manager
* fkie_node_manager: reset package cache on reload in lauch widget
  so you don't need to restart node_manager if new packages are added at
  runtime
* fkie_node_manager: changed behaviour of detailed message box
* fkie_node_manager: fixed clear in echo dialog
* fkie_node_manager: added shortcut Ctrl+R to restart nodes
* Contributors: Alexander Tiderko, Timo Röhling

0.7.7 (2017-10-27)
------------------
* fkie_node_manager: fixed install problem #65
* fkie_node_manager: changed tab order and added Ctrl+Shift+F behaviour
* Contributors: Alexander Tiderko

0.7.6 (2017-10-04)
------------------
* fkie_node_manager: updated version
* fkie_node_manager: editor: removed commented blocks
* fkie_node_manager: editor: fixed un/comment function
* fkie_node_manager: detailed dialog: created own one, enable resize feature
* fkie_node_manager: echo dialog: added a checkbox to dis-/enable message filter
* fkie_node_manager: added log for start and wait for ROS master at the beginning
* fkie_node_manager: fixed utf8 problem with service call
* fkie_node_manager: fixed view problem if ROS_IP is set
* fkie_node_manager: fixed crash while navigation in launch editor
* fkie_node_manager: convert error messages to utf-8
* fkie_node_manager: fixed a lot of utf8 problems
* fkie_node_manager: do not ask changed files for reload an offline master
* fkie_node_manager: reload global parameter, if ROS master was restarted
* fkie_node_manager: file_watcher: fixed wrong detection for paths in parameter values
* fkie_node_manager: editor: adapt indent to previous line on tab
* fkie_node_manager: editor: ident to preview line on pressed return/enter
* fkie_node_manager: label for decimal length changed
* fkie_node_manager: echo_dialog: added array length and a filter for digits after '.' in arrays
* fkie_node_manager: launch dialog: improved graph view
* fkie_node_manager: launch editor: changed line selection behaviour
* fkie_node_manager: added Ctrl+W to close current tab in launch editor
* fkie_node_manager: event connection between launch editor and graph view
* fkie_node_manager: create complete include graph
* fkie_node_manager: added upperBotton again
* fkie_node_manager: removed uppper Button, use Include Graph instead
* fkie_node_manager: added dock widget with include files overview for launch file editor
* fkie_node_manager: reorganized buttons in launch editor and fixed search for included files
* fkie_node_manager: fixed display not complete node/topic/service name
* fkie_node_manager: fixed icon space in description panel
* fkie_node_manager: added icons
  1. in editor for going to next higher launch file
  2. restart node and reload global parameter of the launch file
* fkie_node_manager: changed behaviour after filter changes
* fkie_node_manager: open upper files and insert these in between
* fkie_node_manager: Tab and Backtab fixed
* fkie_node_manager: size units fixed
* fkie_node_manager: fixed search for included files in editor
* fkie_node_manager: enable / disable upper button
* fkie_node_manager: added upper button to the editor dialog
  opens the file which include the current open launch file
* fkie_node_manager: redesigned echo dialog
* fkie_node_manager: added priority queue for opening output console before all nodes are started
* Contributors: Alexander Tiderko

0.7.5 (2017-07-17)
------------------
* fkie_node_manager: improved echo dialog
  * added combobox for maximal size of a message
  * added status for message size (also avarage)
  * added bandwith calculation
  * added info in status bar for latched topic
  * removed status for "std dev" and "window size"
  * store last messages in echo dialog to show them after some filter was chagned
* fkie_node_manager: new feature - start profiles
  you can save and restore the current state for all hosts.
* fkie_node_manager: added a node 'script_runner.py' to launch scripts in a ROS node
  The node exceutes the script on startup and stay alive. On stop you can
  specify a stop script.
* fkie_node_manager: fixed displayed topics in description panel (for different namespaces)
* fkie_node_manager: fixed the warning about illegal ROS name on open echo dialog
* fkie_node_manager: fixed rate filter in echo dialog
* fkie_node_manager: fixed poweroff host
* fkie_node_manager: fixed the end process
* fkie_node_manager: fix crash while remove history file
* fkie_node_manager: added more error handling for script_runner
* fkie_node_manager: added question on stop profile load
* fkie_node_manager: stops profile loading on close profile status
* fkie_node_manager: moved profile code to new file and added progress bar for profile
* fkie_node_manager: fixed rename of file in the launch history
* fkie_node_manager: added a possibility to delete all logs (select host->rosclean purge in description)
* fkie_node_manager: changed key event handling in launch dock to avoid double events
* fkie_node_manager: fix Ctrl+double click on profile history
* fkie_node_manager: added support for default_cfg in profiles
* fkie_node_manager: store the default configuration nodes for profiles
  currently no support to load the profiles with default configuration!
  User will be informed on save a profile with default configuraion.
* fkie_node_manager: fixed detailed dialog for messages without detailed text
* fkie_node_manager: fixed start nodes by load new profile with same launch files
* fkie_node_manager: fixed save profile after load profile
* fkie_node_manager: added description for online state of a master proxy
* fkie_node_manager: skip update of offline hosts
* fkie_node_manager: fixed the list of closing hosts
* fkie_node_manager: added possibility to resize the details message dialog
* fkie_node_manager: removed handling for Ctrl+C and Ctrl+X, so this shortcut now works in description dock
* fkie_node_manager: fixed call of host url options
* fkie_node_manager: fixed problem with editor in foreground
* fkie_node_manager: changed filter handling for latched topics
* fkie_node_manager: fixed warning about echo of last scrapped message
* fkie_node_manager: use objectName() instead of text()
* multiamster_fkie: fixed installation configuration
* Contributors: Alexander Tiderko

0.7.4 (2017-05-03)
------------------
* fkie_node_manager: updated highlightning in sync dialog
* fkie_node_manager: add tooltip to a filter in echo dialog
* fkie_node_manager: fixed problems with ampersand.
  The ampersand is automatically set in QPushButton or QCheckbx by
  KDEPlatformTheme plugin in Qt5
  [https://bugs.kde.org/show_bug.cgi?id=337491]
  A workaroud is to add
  [Development]
  AutoCheckAccelerators=false
  to ~/.config/kdeglobals
  This fix removes the ampersand manually.
* Contributors: Alexander Tiderko

0.7.3 (2017-04-24)
------------------
* fkie_node_manager: fix crash on start master_discovery
* fkie_node_manager: fixed network discovery dialog
* fkie_node_manager: added "pass_all_args" for highlighter
* fkie_node_manager: fixed crash while stop or start a lot of nodes
* fkie_node_manager: changed font color in echo dialog
* fkie_node_manager: changed default color in description widget
* fkie_node_manager: added a workaround for "CTR mode needs counter parameter, not IV"
* fkie_node_manager: reverted url changes
* fixed warnings in API documentation
* fkie_node_manager: fixed url handling in host control
* Contributors: Alexander Tiderko

0.7.2 (2017-01-27)
------------------
* fkie_node_manager: added a parameter to hide domain suffix in description panel and node tree view
* fkie_node_manager: reverted the cut of domains in hostnames
* Contributors: Alexander Tiderko

0.7.1 (2017-01-26)
------------------
* fkie_node_manager: increased precision for float values in combobox (used by settings)
* fkie_node_manager: fixed editor for kinetic; removed setMargin since it not suported by Qt5
* fkie_node_manager: fixed URLs for some buttons in description panel to use it with Qt5
* fkie_node_manager: added more details on start if no 'screen' is available
* fkie_node_manager: changed supervised_popen initialization to avoid multi subclassing
* fkie_node_manager: added a raise Exception if no terminal is availabe
* fkie_node_manager: raise an error now if 'paramiko' is not available
* fkie_node_manager: fixed startup if a node manager instance already running
* fkie_node_manager: added xterm path for macOS
* fkie_node_manager: remove domain suffix from hostname to avoid name problems
* fkie_node_manager: fixed UnboundLocalError for 'selectedGroups' and 'self._accept_next_update'
* Contributors: Alexander Tiderko, Jason Mercer, Dirk Schulz

0.7.0 (2017-01-09)
------------------
* fkie_node_manager: fixed visualisation of not local nodes
    repaired gui_resources.py for Qt5 compatibility
    restore Qt5 compatibility
* fkie_node_manager: added update/set time dialog to update time with ntpdate or date
* fkie_node_manager: added rosbag record to rqt menu
* fkie_node_manager: copy now all selected nodes, topics, services or parameter names to clipboard by pressing Ctrl+C
* fkie_node_manager: added cursor position number to editor
* fkie_node_manager: added indent before hostname in description panel
* fkie_node_manager: added a colorize_host settings parameter
    the color of the host will be now determine automatically
    you can also set own color for each host by double-click on the
    hostname in description panel.
* fkie_node_manager: fixed error after cancel color selection
* fkie_node_manager: use gradient to set color
* fkie_node_manager: now you can define colors for each robot
* fkie_node_manager: removed a broken import
* fkie_node_manager: fixed: no longer clear the search result on click into editor
* fkie_node_manager: find dialog in xml-editor shows now all results in as list
* fkie_node_manager: added clear button to filder lines in dialogs
* fkie_node_manager: add filter to nodes view
  added also a clear button (also ESC) to all filter lines
* fkie_node_manager: fixed some extended visualization for synced nodes
* Contributors: Alexander Tiderko, Sr4l

0.6.2 (2016-11-12)
------------------
* fkie_node_manager: fixed node view for multiple cores on the same host
* fkie_node_manager: fixed capabilities view
* fkie_node_manager: fixed view of group description by groups with one node
* Drop roslib.load_manifest, unneeded with catkin
* fkie_node_manager: moved controls in group description to the top
* fkie_node_manager: fixed the link to node in group description
* fkie_node_manager: fixed crash while kill screen on remote host
* Contributors: Alexander Tiderko

0.6.1 (2016-10-18)
------------------

0.6.0 (2016-10-12)
------------------
* fkie_node_manager: changed find-replace doalog to dockable widget
* fkie_node_manager: changed highlight colors
* fkie_node_manager: added more info for search error
* fkie_node_manager: fixed: comment lines with less then 4 characters
* fkie_node_manager: fixed: `#49 <https://github.com/fkie/fkie_multimaster/issues/49>`_
* fkie_node_manager: added highlightning for yaml stuff inside of a launch file
* fkie_node_manager: fixed: comment of lines with less then 4 characters in xml editor
* fkie_node_manager: fixed: activation of network window after join from network discovery
* fkie_node_manager: fixed: does not open a second configuration editor for a selected node.
* fkie_node_manager: added: 'subst_value' to xml highlighter
* fkie_node_manager: fixed: network discovery
* fkie_node_manager: comment/uncomment fixed
* fkie_node_manager: fixed: detection of included files
* Contributors: Alexander Tiderko

0.5.8 (2016-09-10)
------------------
* fkie_node_manager: fixed the error occurs while open configuration for a selected node
* Contributors: Alexander Tiderko

0.5.7 (2016-09-07)
------------------
* fix imports for Qt5
* fix issue `#43 <https://github.com/fkie/fkie_multimaster/issues/43>`_ - "cannot import name QApplication"
* Contributors: Alexander Tiderko, Sr4l

0.5.6 (2016-09-01)
------------------
* fkie_node_manager: fixed error "No module named xml_editor"
* Contributors: Alexander Tiderko

0.5.5 (2016-08-30)
------------------
* fkie_node_manager: version in info dialog updated
* fkie_node_manager: changed all buttons of the editor to flat
* fkie_node_manager: changes on xml_editor
  * XmlEditor is renamed to Editor and moved into a subdirectory.
  * xml_edit.py splited to exclude all subclasses
  * Search (replace) dialog is redesigned
* fkie_node_manager: added linenumber to the xmleditor
* fkie_node_manager: fix issue `#40 <https://github.com/fkie/fkie_multimaster/issues/40>`_ and some other Qt5 changes
* fkie_node_manager: changed the comment/uncomment in xml editor
* fkie_node_manager: fixed some highlightning problems in xmleditor
* fkie_node_manager: added shortcuts for "Add tag"-Submenu's
* fkie_node_manager: changed xml block highlighting
* fkie_node_manager: fixed seletion in xmleditor
* fkie_multimaster: changed indent in source code to 4
* fkie_node_manager: added a question dialog before set time on remote host
  Time changes leads to problems on tf tree and may have other unexpected
  side effects
* fkie_node_manager: compatibility to Qt5
* fkie_node_manager: fixed the showed network id
* fkie_node_manager: fixed host identification in node view
* fkie_node_manager: changed hostname detection for decision to set ROS_HOSTNAME
* fkie_node_manager: removed pep8 warnings
* fkie_node_manager: fix local discovery node detection
* fkie_node_manager: changed master_discovery node detection
* fkie_node_manager: fixed pep8 warnings
* fkie_node_manager: removed pylint warnings
* fkie_node_manager: new feature: close tabs in Launch-Editor with middle mouse button
* fkie_node_manager: fixed style warning in xml_editor and capability_table
* fkie_node_manager: fixed clear of configuration nodes
* fkie_node_manager: changed identification of master (now it is only the masteruri without address)
* fkie_node_manager: fix in capability table
* fkie_node_manager: removed '-' from master name generation for ROS master with not default port
* fkie_node_manager: remove the ssh connection if the master goes offline. This avoids timeouts after reconnection
* Contributors: Alexander Tiderko

0.5.4 (2016-04-21)
------------------
* fkie_node_manager: added visualisation for not synchronized topics/services
* fkie_node_manager: add parameter to the order of publisher/subscriber in description dock
  new parameter: 'Transpose pub/sub description'
* fkie_node_manager: changed behaviour of description dock while update info
* fkie_node_manager: fixed deselection of text on context menu
* fkie_node_manager: fixed threading problem while searching for sync interfaces
* Contributors: Alexander Tiderko

0.5.3 (2016-04-01)
------------------
* fkie_node_manager: fix remote start
* Contributors: Alexander Tiderko

0.5.2 (2016-03-31)
------------------
* fkie_node_manager: fixed start process on remote hosts without Qt
* Contributors: Alexander Tiderko

0.5.1 (2016-03-23)
------------------

0.5.0 (2016-03-17)
------------------
New Features:
* fkie_node_manager: the start with different ROS_MASTER_URI sets now the ROS_HOSTNAME environment variable if a new masteruri was selected to start node_manager or master_discovery
* fkie_node_manager: added parameter to disable the highlighting of xml blocks
* fkie_node_manager: added ROS-Launch tags to context menu in XML editor
* fkie_node_manager: mark XML tag blocks
* fkie_node_manager: show the filename in the XML editor dialog title
* fkie_node_manager: close configuration items are now sorted
* fkie_node_manager: the confirmation dialog at exit can be deaktivated to stop all nodes and roscore or shutdown the host you can use the close button of each master
* fkie_node_manager: allow to shutdown localhost
* fkie_node_manager: shows 'advanced start' button also if the selected node laready runs

Fixes:
* fkie_node_manager: fixed print XML content in echo_dialog
* fkie_node_manager: avoids the print of an error, while loads a wrongs file on start of the node_manager
* fkie_node_manager: fixed check of running remote roscore
* fkie_node_manager: fixed problem while echo topics on remote hosts
* fkie_node_manager: changed cursor position in XML editor after open node configuration
* fkie_node_manager: fixed replay of topics with array elements
* fkie_node_manager: fixed close process while start/stop nodes
* fkie_node_manager: fixed namespace of capability groups, fixed the missing leading SEP
* fkie_node_manager: fixed - avoid transmition of some included/changed but not needed files to remote host
* fkie_node_manager: fixed start node after a binary was selected from multiple binaries
* fkie_node_manager: removed "'now' FIX" while publish messages to topics
* fkie_node_manager: fixed log format on remote hosts
* Contributors: Alexander Tiderko

0.4.4 (2015-12-18)
------------------
* fkie_node_manager: fixed republish of array values in paraeter dialog
* fkie_node_manager: reviewed the name resolution
* fkie_node_manager: added an IP to hostname resolution
  it is usefull for detection of automatic master_sync start if an IP was
  entered while start of master_discovery
* fkie_node_manager: added a settings parameter 'start_sync_with_discovery'
  The start_sync_with_discovery determine the default behaviour to start
  master_sync with master_discover or not. This presets the 'Start sync'
  parameter in Start-dialog.
* fkie_node_manager: added an option to start master_sync with master_discovery
* fkie_node_manager: added network ID visualization
* fkie_node_manager: fixed joining from discovery dialog
* fkie_node_manager: fixed discovery dialog, which was broken after changes in master_discovery
* fkie_node_manager: highlighted the sync button in ROS network dock
* Contributors: Alexander Tiderko

0.4.3 (2015-11-30)
------------------
* fkie_node_manager: start rviz now as NO rqt plugin
* fkie_node_manager: fixed the sort of paramerter in `add parameter` dialog
* fkie_node_manager: adapt the chagnes in fkie_master_discovery
* fkie_node_manager: fixed the tooltip of the buttons in the description dock
* fkie_node_manager: stop /master_discovery node before poweroff host to avoid timout problems
* fkie_multimaster: reduced logs and warnings on stop nodes while closing node_manager
* fkie_node_manager: added a new button for call service
* fkie_node_manager: added a "copy log path to clipboard" button
* fkie_node_manager: fixed the displayed count of nodes with launch files in description dock
* fkie_node_manager: fixed errors showed while stop nodes on close
* fkie_multimaster: reduced logging of exceptions
* fkie_node_manager: added poweroff command to the host description
* fkie_node_manager: added tooltips to the buttons in description dock
* fkie_node_manager: replaced some icons
* fkie_node_manager: added advanced start link to set console format and loglevel while start of nodes
* fkie_node_manager: skip commented nodes while open a configuration for a selected node
* fkie_node_manager: fixed xml editor; some lines was hide
* fkie_node_manager: added ctrl+shift+slash to shortcuts for un/comment text in editor
* some small changes in find dialog
* Contributors: Alexander Tiderko

0.4.2 (2015-10-19)
------------------
* fkie_node_manager: added further files to change detection
* fkie_node_manager: fixed parameter dialog for some messages e.g. MarkerArray
* fkie_node_manager: shutdown now all nodes and roscore at exit (if selected)
* fkie_node_manager: changed diagnostic visualization
* fkie_node_manager: propagate the diagnostic color of a node to his group
* fkie_node_manager: update the description of selected node after a diagnostic message is recieved
* fkie_multimaster: added a possibility to set time on remote host
* fkie_node_manager: fixed the comparison of host time difference
* fkie_node_manager: added a warning if the time difference to remote host is greater than a defined value (default 3 sec)
* fkie_node_manager: added ControlModifier to package navigation
  Ctrl+DoubleClick:
  * History file: goto the package of the launch file
  * ..: goto root
  * folder: go only one step down, not until first config file
* fkie_node_manager: changed param template for parameter name in editor
* fkie_node_manager: added log button for remote master_discovery
  * show now only the screen log
* fkie_node_manager: fixed save/load in parameter dialog
* fkie_node_manager: fix load parameter with absolute path
* fkie_node_manager: added more info for error while set a parameter with None value
* fkie_node_manager: added icon for rqt plugin
* fkie_node_manager: fixed error which prevent display info and configuration dialogs
* fkie_node_manager: check now for changes of local binaries and ask for restart if these are changed
* fkie_node_manager: fixed problem while publishing to topic with lists and byte values
* fkie_node_manager: added support diagnostics_agg topic
* fkie_node_manager: added a remote script which does not use qt bindings
* Contributors: Alexander Tiderko

0.4.1 (2015-04-28)
------------------
* fkie_node_manager: fixed error while parsing list of lists in parameter dialog
* fkie_node_manager: added scrollarea for dynamic_reconfigure widget
* fixed the usage of new parameter in node_manager
* fkie_node_manager: fixed binary selection while 'add node'
* fkie_multimaster: fixed double log output
* fkie_node_manager: fix to enable the master list if a master_discavery was started
* fkie_node_manager: fixed recursive search
* fkie_multimaster: added network problem detection on remote hosts
* fkie_node_manager: older paramiko versions does not support get_pty parameter in exce_command
* fkie_node_manager: fixed stdout error while transfer files to remote host
* fkie_node_manager: ignore errors caused on after the echo dialog was closed
* fkie_node_manager: changed the color of illegal ros node names
* Contributors: Alexander Tiderko

0.4.0 (2015-02-20)
------------------
* fkie_multimaster: discovery changed
  * reduced the amount of heartbeat messages for discovery
  * added fallback for environments with multicast problems
* fkie_node_manager: added log_level parameter to all nodes
* fkie_node_manager: fixed syntax highlightning
* fkie_node_manager: fix ssh handler
* fkie_node_manager: parameter changed in dialog "start master discovery"
* fkie_node_manager: fixes in parameter dialog
  * fixed filter in parameter dialog
  * fixed parser of the list values
  * update only changed values in ROS parameter server
* fkie_node_manager: default value for heartbeat changed to 0.5
* fkie_node_manager: improved the discovery dialog to detect masters using new methods
* fkie_node_manager: fixed the button view in the sync dialog
* fkie_node_manager: added a xml and yaml validation on save of a configuration files
* Contributors: Alexander Tiderko

0.3.18 (2015-02-18)
-------------------
* fkie_node_manager: fixed alt+space for context menu in xml editor
* node_maanger_fkie: removed sync+AnyMsg option, it is now sync with all messages
* fkie_node_manager: fix an error printed on close of echo dialog
* fkie_node_manager: fixed some ssh issues
* fkie_node_manager: enabled ssh compression
* fkie_node_manager: store user per host
* fkie_node_manager: added rviz to rqt menu
* fkie_node_manager: show now unknown topic types through the SSH connection
* fkie_node_manager: close running nodes on exit
* fkie_node_manager: fixed bug while creation of a new file in xml editor
* fkie_node_manager: added binary selection dialog to xml editor, if you add a node section using 'add tag' button
* fkie_node_manager: trap the errors printed to stderr in popen
* fkie_node_manager: fixed highlightning in editor
* Contributors: Alexander Tiderko

0.3.17 (2015-01-22)
-------------------
* fkie_node_manager: switch to local monitoring after connection problems to local master_discovery
* fkie_node_manager: added an update procedure to refresh discovered masters
  In same cases the messages, which are send on the shutdown of the
  master_discovery are not received by node_manager. To update the
  discovered list in node_manager the complete list of discoevered hosts
  will be requested, if the localhost master is added as new master.
* fkie_node_manager: fixed error while publishing to 'std_msgs/Empty'
* Contributors: Alexander Tiderko

0.3.16 (2014-12-08)
-------------------
* fkie_node_manager: fixed a problem with screen view
  The node_manager uses the /usr/bin/x-terminal-emulator to show the
  screen content of the nodes. To execute a command with arguments
  'konsole', 'xterm' uses -e, 'terminator', 'gnome-terminal' or
  'xfce4-terminal'use '-x'.
* Contributors: Alexander Tiderko

0.3.15 (2014-12-01)
-------------------
* fkie_node_manager: fixed sync button handling
* fkie_multimaster: removed some python mistakes
* fkie_node_manager: removed some python mistakes
* fkie_node_manager: fixed node selection in description dock
* fkie_node_manager: some icons changed
* fkie_node_manager: 'autoupdate' parameter added
  The autoupdate parameter disables the automatic requests. It is usefull
  for low bandwidth networks.
* fkie_node_manager: reduced remote parameter requests
* fkie_node_manager: added a republish functionality
  This function is accessible in extended info widget.
* fkie_node_manager: fix publish with rate slower one
  Updated the topic info. Added constants to message definition view.
* fkie_node_manager: restores the view of expanded capability groups after reload of a launch file
* node_managef_fkie: fix sidebar parameter selection
* fkie_node_manager: fixes in parameter dialog
  * fixed filter in parameter dialog
  * fixed parser of the list values
  * update only changed values in ROS parameter server
* Contributors: Alexander Tiderko

0.3.14 (2014-10-24)
-------------------
* fkie_node_manager: added a warning to capability table, if multiple configurations for the same node are loaded
* fkie_node_manager: remove now the configuration in capability table after a host was removed
* fkie_node_manager: fixed error while navigate in description panel
* fkie_node_manager: changed sidebar parameter handling (for start host dialog)
* fkie_node_manager: changed the handling on click the sync button in master list
* fkie_node_manager: fixed tooltip for recent loaded files
* fkie_node_manager: fixed problems in capability table with multi-launch-files for the same host and group
* CapabilityHeader: Keep indices of _data and controlWidget in sync when inserting new capabilities
* Fixed crash in master_list_model if IPv6 addresses are present on the host
* fkie_node_manager:manual link added
* fkie_node_manager: added args and remaps to change detection after reload a launch file
* fkie_node_manager: ignore namespace while display the Capabilities in Capabilities table
* fkie_node_manager: fixed some template tags in xml editor
* fkie_node_manager: stop nodes first while restart nodes after loading a launch file
* fkie_node_manager: added support of $(find ...) statement to add images in decription of capabilities
* fkie_node_manager: xmleditor - ask for save by pressing ESC
* fkie_node_manager: changed the update strategy for description dock
* fkie_node_manager: changed the update strategy for description dock
* fkie_node_manager: changed name creation for default configuration node
* fkie_node_manager: fixed blocked focus if a xmleditor was open
* fkie_node_manager: fixed highlighter problem in pyqt
* fkie_node_manager: improved respawn script
* fkie_node_manager: fixed handling of history files
* fkie_node_manager: mark line with problems in launch editor
* Contributors: Alexander, Alexander Tiderko, Stefan Oßwald, Timo Röhling

0.3.13 (2014-07-29)
-------------------
* fkie_node_manager: fixed the button view in the sync dialog
* fkie_node_manager: added a xml and yaml validation on save of a configuration files
* fkie_node_manager: changed the navigation in info widget
* fkie_node_manager: raise launch dock after the settings are restored
* fkie_node_manager: show up directory while package selection
* fkie_node_manager: added comment/uncomment functionality
* fkie_node_manager: added caching for browsing in launch files
* fkie_node_manager: show also folder with additional config files
* fkie_node_manager: stores the xml editor geometry
* Contributors: Alexander Tiderko

0.3.12 (2014-07-08)
-------------------
* fkie_node_manager: fix instalation problem with missed .ui files
* fkie_node_manager: fixed ros master preparation
  Do not try to start ROS master on remote hosts for echo topics, if this
  host are not reachable.
* Contributors: Alexander Tiderko

0.3.11 (2014-06-04)
-------------------
* fkie_node_manager: replaced the rxconsole and rxgraph by rqt button to start rqt plugins related to selected master
* fkie_node_manager: added a setting docking window
* fkie_node_manager: hints on start problems fixed, if no screen is installed
* fkie_node_manager: added a dock widget and button which shows warning messages
* fkie_node_manager: select the topics and services of a node while tab change and not while node selection. This reduce the cpu load.
* fkie_node_manager: fixed detection of local host at start
* fkie_node_manager: fix the removing of local master at startup
* fkie_node_manager: added features to launch file view
  * Search for packages
  * rename files
  * copy files
* fkie_node_manager: do not wait in the discovery loop at shutdown
* fkie_node_manager: cancel buttons redesined, some titles renamed
* fkie_node_manager: reduced the displayed namespace of the topics and services in info area
* fkie_node_manager: added F4 and F3 shortcasts for aditing a configuration and show a screen of a node
* fkie_node_manager: fixed InteractionNeededError while starting nodes on remote hosts using run dialog.
* fkie_node_manager: added timestamps to each printed message
* fkie_node_manager: fix detailed message box. Close using ESC button.
* fkie_node_manager: reload root path in xml file view, if the current path was deleted
* fkie_node_manager: fixed include tag of dropped file in xml editor
* fkie_node_manager: added for each node respawn parameters
* fkie_node_manager: improve respawn script
  The new script correctly checks the exit code of the launched
  process and can limit the number of respawns for faulty
  nodes.
* fkie_node_manager: use -T for terminal emulator
  -T is compatible with more terminal emulators than -title
* fkie_node_manager: added handling for some of other configuration file types to launch file view
* Open terminal windows with x-terminal-emulator
  The /usr/bin/x-terminal-emulator symlink is available on Debian
  based systems and points to the default terminal emulator on
  the system. /usr/bin/xterm will be used as fallback.
* fkie_node_manager: changed side bar selection while start hosts
* fkie_node_manager: fixed the parameter handling of parameter with list type
* Contributors: Alexander, Alexander Tiderko, Sammy Pfeiffer, Timo Röhling

0.3.10 (2014-03-31)
-------------------
* fkie_node_manager: fixed the activation of the local monitoring. Fixed the cancelation in selection dialog.
* fkie_node_manager: added an indicator for running roslaunch server
* fkie_node_manager: fixed layout problems
* fkie_node_manager: dialog size of `start master_discovery` changed
* fkie_node_manager: added a side bar with checkitems in start host dialog
* fkie_node_manager: fixed remove entries in combonox of parameter dialog
* fkie_node_manager: remove comments in launch file fixed
* fkie_node_manager: added a check for changed files in parameter value
* fkie_node_manager: inform about changed files only on activating the main GUI
* fkie_node_manager: fixed search routine
* fkie_node_manager: fixed multiple entries in dialog for publishing to a new topic
* fkie_node_manager: added a context sensitive proposals in XML editor
* fkie_node_manager: enabled drag&drop action in xmleditor and launch view
* fkie_node_manager: added a button for quick insertion of launch tags
* fkie_node_manager: reduced the cpu load of echo dialog
* fkie_node_manager: added a line limit in echo dialog
* fkie_node_manager: fixed the processing of jobs after the `cancel` button was pressed
* fkie_node_manager: added a `reload global parameter` link
  - select the loaded row in launch dialog after loading the launch file
  with double click
* fkie_node_manager: fixed start nodes with ns
* node_maager_fkie: the launch files are now loaded in a thread, so they don't block
* fkie_node_manager: fixed duplicate detection of running and synchronized nodes
* fkie_node_manager: sync dialog extended by a new button to sync topics containing AnyMsg as type
* fkie_node_manager: cmd line output for registered parameter changed
* fkie_node_manager: removed project file
* fkie_node_manager: remember the used path in parameter dialog
* fkie_node_manager: changed the handling of localhost in machine tag of launchfile

0.3.9 (2013-12-12)
------------------
* fkie_node_manager: set node to warning state, if it not renning propertly because of problems with illegal name
* fkie_node_manager: fixed detailed_msg_box error
* fkie_node_manager: added highlighting for illegal ros names
* fkie_multimaster: moved .gitignore to top level

0.3.8 (2013-12-10)
------------------
* fkie_node_manager: added support for /robot_icon parameter to show an image of the roboter
* fkie_node_manager: fixed handling of binary data in ROS parameter server
* fkie_node_manager: update robot image on cancel file selection dialog
* fkie_node_manager: can now change the robot image by double-click on robot image
* fkie_node_manager: added autoselect corresponding topics and services on node selection
* fkie_node_manager: reduced timestamp updates, if node_manager is not active
* fkie_multimaster: added a possibility to deaktivate the multicast heart bearts
* fkie_node_manager: selection dialog extended by an description label
* fkie_node_manager: handling of included files chagned, to avoid errors if a package was not found
* fkie_node_manager: buttons of the discovery widged chagned
* fkie_node_manager: control buttons redesigned
* fkie_node_manager: added 'Do not display this warning again' button to warning message
* fkie_node_manager: fixed deleting of not reachable hosts
* fkie_node_manager: fixed wrong reference in sync_dialog
* fkie_node_manager: fixed copy mode (Ctrl+C copy now first column, Ctrl+X: type or value)
* fkie_node_manager: update launch file view after loading launch file
* fkie_node_manager: fixed echo dialog (icons, additional info)
* fkie_node_manager: added ROS_NAMESPACE environment parameter to launch process to handle some cases, e.g. rqt_cpp plugins
* fkie_node_manager: fixed watching for changes in included files
* fkie_node_manager: Delete key deletes now the selected history launch file
* fkie_node_manager: reduced window size
* fkie_node_manager: ignore empty 'capability_group' values
* fkie_multimaster: catkin_lint inspired fixes, thanks @roehling
* fkie_node_manager: fixed help call in the console
* fkie_node_manager: fix detection for included files
* fkie_node_manager: fixed open sync dialog from info panel
* fkie_node_manager: added a yaml highlighter
* fkie_node_manager: argparse integrated
* fkie_node_manager: fixed lower compare of topic and service names
* fkie_node_manager: fix - use now sensetive comparison of node names
* fkie_node_manager: fixed launch file browsing
* fkie_node_manager: fixed skipped display messages on latched topics

0.3.7 (2013-10-17)
------------------
* fkie_node_manager: fixed start button description
* fkie_node_manager: added an info button
* fkie_node_manager: changed calling of sync dialog
* fkie_node_manager: showing duplicate nodes fixed
* fkie_multimaster: fixed problems with resolving service types while sync
  while synchronization not all topics and services can be synchronized
  because of filter or errors. A detection for this case was added.
* fkie_node_manager: added user selection for remote hosts
* fkie_node_manager: fixed some paths
* fkie_node_manager: added SAVE and LOAD buttons to parameter dialog
* fkie_node_manager: fixed start nodes in multimaster on the same host
* fkie_node_manager: replaced the sync checkbox in masterlist by a sync icon
* fkie_node_manager: fixed filtering topics, services and parameter
* fkie_node_manager: buttons resized
* fkie_node_manager: added missed start parameter to master_sync
* fkie_node_manager: removed some unneeded borders in gui
* fkie_node_manager: fix loading launch file
* fkie_node_manager: fixed parameter groups
* fkie_node_manager: added new interface of dynamic_reconfigure
* fkie_node_manager: show node_manager window maximized, if the screen is small
* fkie_node_manager: fixed raise conditions
* fkie_node_manager: added filter to selected dialog and changed selection behavior
* fkie_node_manager: fix node matching
* fkie_node_manager: fixed absolute path in env of the launch file

0.3.6 (2013-09-17)
------------------
* fkie_node_manager: added a notifiaction, if `use_sim_time` parameter is set to true
* fkie_node_manager: added some control elements to node/host description
* fkie_node_manager: fix load launch file
* fkie_node_manager: fix filter in paramter dialog
* fkie_node_manager: fixed do not store the launch file on error
* fkie_node_manager: the minimum size of the parameter dialog increased
* fkie_node_manager: update the capability group of the node using the ROS parameter server, if no launch file is loaded
* fkie_node_manager: fixed cancel loading of the launch file, on cancel input args
  fkie_node_manager: do not restart anonymous nodes on relaod launch file
  fkie_node_manager: fixed closing of the remote default configs on same host but other roscore
* fkie_node_manager: resize the node_manager window on small
* fkie_node_manager: changed the intepretation of the group description
* fkie_node_manager: remove not existing remote node information. In case of restarting a ROS node without stopn a running node.
* fkie_node_manager: fixed buttons description
* fkie_node_manager: fixed change detection in included files
* fkie_node_manager: add detection of changes in the reloaded launch file and restart affected nodes
* fkie_node_manager: fixed clear_params

0.3.5 (2013-09-06)
------------------
* fkie_node_manager: fixed launch selection for favirites with same launch file name
* fkie_node_manager: fixed process id view of nodes for multiple sync hosts

0.3.4 (2013-09-05)
------------------
* fkie_node_manager: fixed file paths (removed warnings in file_watcher)
* fkie_node_manager: clear cached package names on refreshing launch file view
* fkie_node_manager: capability_group parameter can now be defined in a namespace
* fkie_node_manager: fixed pakage_name result
  added caching for package_name results

0.3.3 (2013-09-04)
------------------
* fkie_node_manager: Parse package.xml for name
  Although package folders should have the same name as the
  package, some packages (e.g. swig-wx) violate this.
  Thus, we use catkin_pkg.package.parse_package to parse
  the package.xml and look for the <name> tag, which
  contains the correct package name.
* fkie_node_manager: Install data files without executable bit
* fkie_node_manager: added a button to hide the dock widgets
* fkie_node_manager: added a question dialog to start the synchronization with a loaded config, if any exists
* fkie_node_manager: increased timeout for transfer of parameter while start of nodes
* fkie_node_manager: fixed node name creation for publishing of topics
* fkie_node_manager: fixed start of master_sync with interface file
* fkie_node_manager: removed some exeption for pyqt workaround
* fkie_node_manager: added a warning in paramter dialog
* fkie_node_manager: fixed names, preselect all files to reload after a file was changed
* fkie_node_manager: added a buttons to save and load configurations
* fkie_node_manager: show the parent of the src-folder
* fkie_node_manager: plugin renamed
* fkie_node_manager: fixed finish function to stop the running timer
* fkie_node_manager: file watcher updated, changes now notified once for all master
* fkie_multimaster: .gitignore changed
* fkie_node_manager: don't ask for argv's while reloading
* fkie_node_manager: fixed a problem while launching a default cfg nodes
* fkie_node_manager: searching for packages in rundialog after dialog opened
* fkie_node_manager: fixed waiting for roscore
* fkie_node_manager: added the default group for system nodes, fixed an often update problem
* fkie_node_manager: fixed problem while openning an editor
* fkie_node_manager: increased the wait for ROS Master
* fkie_node_manager: added the possibility to enter a varible count of list entries while calling a service or publishing to a topic
* fkie_node_manager: changed the handling while close multiple configurations
* fkie_node_manager: added the parameter as pkg:// URL to launch a default_cfg at start of node_manager
* fkie_multimaster: (*) added additional filtered interface to master_discovery rpc-server to get a filtered MasterInfo and reduce the load on network.
  (*) added the possibility to sync remote nodes using ~sync_remote_nodes parameter
* fkie_node_manager: added a possibility to create a new files
* fkie_node_manager: fixed error while browsing in launch files
* fkie_node_manager: (1) added a button to transfer launch files to remote machines,
  (2) upgraded the editor for sync dialog
  (3) added more info to progress bars
* fkie_node_manager: limited displaying frequency for echo dialog
* fkie_node_manager: limited the displayed messages in echo widget
