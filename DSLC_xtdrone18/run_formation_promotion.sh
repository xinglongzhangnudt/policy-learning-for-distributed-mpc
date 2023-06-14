#!/bin/bash
python3 leader.py 'iris' 18 &
python3 avoid.py 'iris' 18  'vel' &
python3 e1_pub.py 'iris' 18 'vel' &
uav_id=1
while(( $uav_id< 18 )) 
do
    python3 follower_consensus_promotion.py 'iris' $uav_id 18 'vel' &
    let "uav_id++"
done