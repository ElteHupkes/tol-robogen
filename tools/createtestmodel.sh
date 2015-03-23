#/bin/bash
../build/test simpleRobot.txt ./models/temp_bot/model_raw.sdf 10.0
xmllint --format ./models/temp_bot/model_raw.sdf > ./models/temp_bot/model.sdf
