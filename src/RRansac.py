
import numpy as np
import sys

class RRansac:
	''' Use the fit a model using RANSAC '''

	def __init__(self, model, threshold):
		self.maxIterations = 10000
		self.fractionPretest = 0.05 # 5%
		self.probability = 0.99

		self.model = model
		self.threshold = threshold

		self.bestCoeff = np.zeros(4)
		self.bestModel = np.array([])

	def computeModel(self, warmStart=None, constraint=None):
		selection = np.array([])
		bestInliersCount = 0
		bestModel = None
		bestCoeff = [0.,0.,1.,0.]
		bestInliers = np.array([])

		nPresample = int(self.fractionPretest * len(self.model.indices))
		k = 1.0
		it = 0
		skipped = 0
		maxSkipped = 5*self.maxIterations
		coeff = np.zeros(4)

		while it < k and it < maxSkipped and skipped < maxSkipped:

			if it == 0 and warmStart != None:
				coeff = warmStart
			else:
				selection = self.model.getSamples()
				if len(selection) == 0: break # Couldn't get any good samples

				# Compute the coefficients given the samples
				success, coeff = self.model.computeModelCoefficients(selection)
				if not success:
					skipped += 1
					continue

			# As part of the R-RANSAC algorithm, we presample a random subset
			# of the point cloud, and check if it matches the hypothesis.
			pretestSamples = self.model.getRandomSamples(nPresample)

			if not constraint(coeff) or not self.model.doSamplesVerifyModel(\
				pretestSamples,coeff,self.threshold):
				# k needs to be set on the first iteration
				if k > 1.0:
					it += 1
					continue

			success, inliers = self.model.selectWithinDistance(\
				coeff, self.threshold)
			nInliers = len(inliers)


			if nInliers > bestInliersCount:
				bestInliersCount = nInliers
				bestModel = selection
				bestCoeff = coeff
				bestInliers = inliers

				# Compute k parameter (k=log(z)/log(1-w^n))
				w = float(nInliers)/len(self.model.indices)
				powOutliers = 1.0 - w**len(selection)
				powOutliers = max(powOutliers, sys.float_info.epsilon)
				powOutliers = min(powOutliers, 1 - sys.float_info.epsilon)
				k = np.log(1 - self.probability) / np.log(powOutliers)

			it += 1

			if it > self.maxIterations: break

		#print 'Done in', it, 'iterations'
		if len(bestInliers) == 0:
			inliers = np.array([])
			return False, np.array([0.,0.,1.,0.]), np.array([]), np.array([])

		# Fix it up a little....
		if len(bestInliers) > 4:
			# Select a subset for efficiency
			inliers = self.model.getRandomSamples(500, bestInliers)
			# Refit
			success, newCoeff = self.model.optimiseModelCoefficients(\
				inliers,bestCoeff)
			if success:
				# Reselect
				success, newInliers = self.model.selectWithinDistance(\
					bestCoeff,self.threshold)
				if success:
					bestCoeff = newCoeff
					bestInliers = newInliers

		return True, np.array(bestCoeff), bestInliers, bestModel





