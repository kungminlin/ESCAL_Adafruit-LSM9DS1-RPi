import matplotlib.pyplot as plt
import matplotlib.animation as animation
import datetime

class Dataplot:
	def __init__(self, fig):
		self.fig = fig
		self.plots = {}

	def add_subplot(self, name, title):
		ax = self.fig.add_subplot(1, 1, 1)
		x = []
		y = []
		self.plots[name] = {'title': title, 'ax': ax, 'x': x, 'y': y, 'plot': None, 'datasource': None}

	def get_plots(self):
		return self.plots

	def update_data(self, name, data):
		x.append(datetime.datetime.now().strftime('%H:%M:%S:%f'))
		y.append(data)
		x = x[-20:]
		y = y[-20:]
		self.plots[name]['plot'].set_data(x, y)
		# ax.clear()

	def animate_helper(self):
		for key, value in self.plots.items():
			value['plot'].set_data(x.append(datetime.datetime.now().strftime('%H:%M:%S:%f')), value['datasource'])
			value['x'] = value['x'][-20:]
			value['y'] = value['y'][-20:]
			value['ax'].clear()


		print('hi')

	def animate(self, func):
		ani = animation.FuncAnimation(self.fig, self.animate_helper(), interval=20)