#!/bin/bash
python3 leader.py 'iris' 6 &
python3 avoid.py 'iris' 6  'vel' &
python3 e1_pub.py 'iris' 6 'vel' &
uav_id=1
while(( $uav_id< 6 )) 
do
    python3 follower_consensus_promotion.py 'iris' $uav_id 6 'vel' &
    let "uav_id++"
done