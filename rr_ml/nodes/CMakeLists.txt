catkin_install_python(PROGRAMS
        run_e2e
        train_e2e
        convert_data_e2e
        train_seg
        label_folder
        label_image
        bag_image_extractor
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
