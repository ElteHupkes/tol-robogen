#/bin/bash
../build/test simplerRobot.txt ./models/temp_bot/model_raw.sdf 1.0
xmllint --format ./models/temp_bot/model_raw.sdf > ./models/temp_bot/model.sdf
