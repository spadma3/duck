import time

def now():
	return int(time.time() * 1000)

def stdout(entity, message):
	print '[%s] :: %s' % ( entity, message )

def publish( lcm_handler, channel, message, n_copies=2 ):
    if n_copies == 'default':
		n_copies = 2
    for i in range(n_copies):
        lcm_handler.publish( channel, message )
        time.sleep( 0.01 )
