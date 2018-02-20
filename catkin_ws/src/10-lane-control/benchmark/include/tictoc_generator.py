import time

class TicTocGenerator():
    # Generator that returns time differences
    def __init__(self):
        self.ti = 0           # initial time
        self.tf = time.time() # final time
        while True:
            self.ti = self.tf
            self.tf = time.time()
            self.diff = self.tf-self.ti # returns the time difference

        # self.TicToc = TicTocGenerator() # create an instance of the TicTocGen generator

# This will be the main function through which we define both tic() and toc()
    def toc(tempBool=True):
        # Prints the time difference yielded by generator instance TicToc
        self.tempTimeInterval = self.diff
        if tempBool:
            print( "Elapsed time: %f seconds.\n" %tempTimeInterval )

    def tic():
        # Records a time in TicToc, marks the beginning of a time interval
        toc(False)