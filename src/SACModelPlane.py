import numpy as np
import random

class SACModelPlane:
    '''
    A Sample Consensus Model for a plane in 3D space.
    The model is constrained such that the plane's normal must not deviate
    from a specified vector by more than a specifed angle.
    (Adapted from the PCL Source (github.com/PointCloudLibrary/pcl).)
    '''

    modelCoeff = np.array([0,0,0,0], dtype=float)
    samples = np.array([])
    def __init__(self, cloud):
        '''
        Cloud is a 1D or 2D array of 3D points (2D or 3D matrix).
        These points are used to form the model.
        '''
        # Let extract the points that are not nan
        mask = np.logical_not( np.any( np.isnan(cloud), axis=-1 ) )
        self.points = cloud[mask]
        self.indices = np.arange(len(self.points))

        self._ind2coord = np.where(mask)
        self._coord2ind = -np.ones(mask.shape)
        self._coord2ind[mask] = np.arange(len(self._ind2coord[0]))

    def ind2coord(self, ind):
        return (self._ind2coord[0][ind], self._ind2coord[1][ind])

    def coord2ind(self, coord):
        '''
        Given an Nx2 array containing coordinate points in an image, return the
        indices of the corresponding points in the 3D point cloud. Coordinates
        to pixels that have no depth data will simply be ignored.
        '''
        ind = self._coord2ind[coord[:,0], coord[:,1]]
        ind = ind[ind!=-1]
        return ind.astype(int)

    def getRandomSamples(self, n, subset=None):
        if subset == None or len(subset) < n: subset = self.indices
        
        if len(self.indices) < n: return self.indices
        return random.sample(subset, n)

    def getSamples(self, subset=None, maxIter=1000):
        '''
        Get 3 unique samples that form a valid plane from the point cloud.
        maxIter defines the maximum number of samples made before the algorithm
        gives up.
        '''
        for i in range(maxIter):
            sample = self.getRandomSamples(3, subset)
            # Check that the samples are goodly
            if self.isSampleGood(sample): return sample

        return np.array([])


    def isSampleGood(self, sampleIndices):
        # Check length
        if not len(sampleIndices) ==3:
            return False

        # Check indices
        samples = []
        try:
            samples = self.points[sampleIndices]
        except IndexError:
            return False

        # Check colinearity
        A,B,C = tuple(samples)

        dyBdyC = (B-A)/(C-A)

        return (dyBdyC[0] != dyBdyC[1]) or (dyBdyC[1] != dyBdyC[2])

    def computeModelCoefficients(self, sampleIndices):
        '''
        Obtain the model coefficients (x,y,z,d) of a plane from the points
        referred to by sampleIndices.
        SampleIndices should be a list of 3 indices referring to the points 
        sampled.
        Return (Success, np.array[x,y,z,d]  )
        Where (x,y,z) is a unit vector describing the plane's normal.
        '''
        badIndices = False

        if not len(sampleIndices) == 3:
            badIndices = True
        else:
            try:
                self.samples = self.points[sampleIndices]
            except IndexError:
                badIndices = True

        if badIndices:
            print "Bad Indices!", sampleIndices
            print "Should be a list of 3 indices. E.g. [0, 4, 2]"
            return False, np.zeros(4)

        A,B,C = tuple(self.samples)
        # Get the segments connecting A->B, A->C
        AB = B - A
        AC = C - A

        # Check for colinearity
        colinear = True
        nonZeroInd = []
        for i in range(3):
            # Avoid divide by zero
            if AC[i] == 0:
                if AB[i] != 0:
                    colinear = False
                    break
            else:
                nonZeroInd.append(i)

        if colinear:
            dyBdyC = AB[nonZeroInd]/AC[nonZeroInd]
            if len(nonZeroInd) > 1:
                if dyBdyC[0] != dyBdyC[1]:
                    colinear = False
                elif len(nonZeroInd) > 2 and dyBdyC[1] != dyBdyC[2]:
                    colinear = False
        
        if colinear:
            return False, np.zeros(4)
                

        # Compute the plane coefficients from the 3 given points
        # Plane normal n = (B - A) x (C - A) = cross(AB, AC)
        norm = np.cross(AB, AC)
        # Normalize it
        norm /= np.linalg.norm(norm)

        # We calculate the residual of the current cooefs (... + d = 0)
        d = -np.dot(norm, A)

        self.modelCoeff[0:3] = norm
        self.modelCoeff[3] = d

        return True, self.modelCoeff

    def getDistancesToModel(self, modelCoeff, subset=None, signed=False):
        '''
        Get the distances of each point in the cloud to the model.
        Return (Success, distances)
        Where distances is a numpy array
        '''
        if len(modelCoeff) != 4:
            print "Model coefficients should be of length 4. Received:",\
                modelCoeff
            return False, np.array([])
        norm = np.array( modelCoeff[0:3] )
        L2 = np.sum( norm**2 )
        if L2==0:
            return False, np.array([])
        if L2!=1:
            modelCoeff /= np.sqrt(L2)

        if subset == None: subset = self.indices
        N = len(subset)

        if N==0:
            print "getDistancesToModel: Empty Subset!"
            return False, np.array([])

        coeffTiled = np.tile(modelCoeff, (N,1))
        # Homogenious coords
        pointsHom = np.column_stack( (self.points[subset], np.ones(N)) )
        # Vectorize the dot product
        distances = np.sum( pointsHom*coeffTiled, axis=1 )
        if not signed: distances = np.abs(distances)

        return True, distances

    def selectWithinDistance(self, modelCoeff, threshold):
        '''
        Find all points which are within a certain distance to the model.
        Return (Success, inliers)
        Where inliers is a numpy array which indexes the selected points.
        '''
        success, distances = self.getDistancesToModel(modelCoeff)

        if not success:
            return False, np.array([])

        inliers = np.where(distances <= threshold)[0]
        return True, inliers

    def optimiseModelCoefficients(self, inliers, modelCoeff):
        '''
        Optimise the model coefficients given the selected inliers.
        Return (Success, newModelCoeff)
        '''
        if len(modelCoeff) != 4:
            print "Model coefficients should be of length, 4"
            print "Received:", modelCoeff
            return False, modelCoeff

        if  len(inliers) < 4:
            print "Need at least 4 inliers to predict a new model."
            return False, modelCoeff

        newModelCoeff = self.fitPlaneSVD(self.points[inliers])
        return True, newModelCoeff


    def fitPlaneSVD(self, points):
        N = len(points)
        # Set up constraint equations of the form  AB = 0,
        # where B is a column vector of the plane coefficients
        # in the form b(1)*X + b(2)*Y +b(3)*Z + b(4) = 0.
        pointsHom = np.column_stack( (points, np.ones(N)) )
        [u, d, v] = np.linalg.svd(pointsHom,0)

        #print pointsHom.shape
        #print v
        modelCoeff = v[3,:];                    # Solution is last column of v.
        nn = np.linalg.norm(modelCoeff[0:3])
        modelCoeff = modelCoeff / nn
        return modelCoeff

    def doSamplesVerifyModel(self, sampleIndices, modelCoeff, threshold):
        samples = self.points[sampleIndices]
        S = len(samples)
        if S == 0: return False

        samplesHom = np.column_stack( (samples, np.ones(S)) )
        coeffTiled = np.tile(modelCoeff, (S,1))
        distances = np.sum( samplesHom*coeffTiled, axis=1)

        if len(np.where(distances<=threshold)[0]) < S:
            return False
        else:
            return True

def main():
    cloud = np.array([[0,0,1], [1,0,0], [0,1,0]], dtype=float)
    smp = SACModelPlane(cloud)
    success, coeff = smp.computeModelCoefficients([0,1,2])
    print success, coeff

    success, inliers = smp.selectWithinDistance(coeff, 0.1)
    print success
    print inliers

    cloudbig = np.array([[0,0,1], [1,0,0], [0,1,0], \
        [1,1,1],[0.5,0.5,0]], dtype=float)
    coeff = smp.fitPlaneSVD(cloudbig)
    print coeff

if __name__ == '__main__':
    main()







