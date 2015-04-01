#/bin/bash
for i in 1 2 3
do
    for j in 15 30 45 60 75 
    do
        ./createworld.php $i $j 3 > stress-$i/test_$j.world
    done
done
