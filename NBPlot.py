import ProcessPlotter
try:
    from multiprocessing import Process, Pipe
except ImportError:
    from processing import Process, Pipe

class NBPlot(object):
    def __init__(self):
        self.plot_pipe, plotter_pipe = Pipe()
        self.plotter = ProcessPlotter.ProcessPlotter()
        self.plot_process = Process(target=self.plotter, args=(plotter_pipe,))
        self.plot_process.daemon = True
        self.plot_process.start()

    def plot(self, data, finished=False):
        send = self.plot_pipe.send
        if finished:
            send(None)
        else:
            send(data)