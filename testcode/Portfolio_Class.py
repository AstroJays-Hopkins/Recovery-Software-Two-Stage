import pandas as pd

class Portoflio(object):
	"""docstring for Portoflio"""
	def __init__(self, asset_list = [], X0 = []):
		self.data = pd.read_csv("ETFs_adjclose Feb162024.csv")
		self.asset_list = asset_list
		if sum(X0)!=1:
			raise ValueError("100% of the PF must be invested")
		self.holdings = {k: v for k, v in zip(asset_list, X0)}
		self.total_value = self.compute_value(0)

	self.compute_value(self,time_stamp):
		print(self.data)
		

test = Portoflio()