import push
import time

#push.demo()
#push.demo_vn()
#push.demo_single_random()
start = time.time()
push.demo_push_poses()
end = time.time()
print('Time elapsed: %d' %(end-start))
# push.visualize_dd()
