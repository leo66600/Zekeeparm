cmake_minimum_required(VERSION 3.22)
project(zekeep_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/worktable_collision.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME worktable_collision
)

install(PROGRAMS scripts/pick_and_place_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pick_and_place_node
)

install(PROGRAMS scripts/training_box_collision.sh
  DESTINATION lib/${PROJECT_NAME}
  RENAME training_box_collision
)

ament_package()
