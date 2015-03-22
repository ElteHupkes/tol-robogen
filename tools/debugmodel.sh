#!/bin/bash
./test > ~/.gazebo/models/temp_bot/model_raw.sdf
xmllint --format ~/.gazebo/models/temp_bot/model_raw.sdf > ~/.gazebo/models/temp_bot/model.sdf
