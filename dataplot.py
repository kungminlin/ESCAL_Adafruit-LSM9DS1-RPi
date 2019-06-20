import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Dataplot:
	def __init__(self, limit=20):
		self.limit = limit
		self.plots = {}

	def add_subplot(self, name, title=None, xlabel=None, ylabel=None):
		ax = plt.subplot(2, 1, len(self.plots)+1)
		plt.xlabel = xlabel
		plt.ylabel = ylabel
		ax.title.set_text(title)
		self.plots[name] = {'ax': ax, 'x': [], 'y': []}

	def update_data(self, name, x, y):
		self.plots[name]['x'].append(x)
		self.plots[name]['x'][-self.limit:]
		self.plots[name]['y'].append(y)
		self.plots[name]['y'][-self.limit:]
		print(self.plots[name]['x'])
		print(self.plots[name]['y'])
		# self.plots[name]['plot'].set_ydata()
		# plt.draw()
		self.plots[name]['ax'].clear()
		self.plots[name]['ax'].plot(self.plots[name]['x'], self.plots[name]['y'], '.')

	def get_plots(self):
		return self.plots