from utilities.encoders import Encoders
from utilities.initialize import Initialize 
from multiprocessing import Process


initialize = Initialize()
initialize.startup()

encoders = Encoders()

p = Process(target=encoders.get_encoder_ticks)
p.start()
p.join()

while(1):
    print(encoders.counter_left.value)