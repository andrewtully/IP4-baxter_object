""" http://mediabox.grasp.upenn.edu/svn/penn-ros-pkgs/tfpl_stack/trunk/perception_py/src/cameramodels.py """
import array

import cv
import sensor_msgs.msg
import math
import numpy as np 
#import cvbf

def mkmat(rows, cols, L):
    mat = cv.CreateMat(rows, cols, cv.CV_64FC1)
    cv.SetData(mat, array.array('d', L), 8 * cols)
    return mat

#def cv2kdl_rot(cv_rot):
#    return kdl.Rotation(cv_rot[0,0],cv_rot[0,1],cv_rot[0,2], \
#                        cv_rot[1,0],cv_rot[1,1],cv_rot[1,2], \
#                        cv_rot[2,0],cv_rot[2,1],cv_rot[2,2])

class PinholeCameraModel:

    """
    A pinhole camera is an idealized monocular camera.
    """

    def __init__(self):
        pass

    def fromCameraInfo(self, msg):
        """
        :param msg: camera parameters
        :type msg:  sensor_msgs.msg.CameraInfo

        Set the camera parameters from the :class:`sensor_msgs.msg.CameraInfo` message.
        """
        self.K = mkmat(3, 3, msg.K)
        self.D = mkmat(4, 1, msg.D[:4])
        self.R = mkmat(3, 3, msg.R)
        self.P = mkmat(3, 4, msg.P)
        self.width = msg.width
        self.height = msg.height
        self.frame_id = msg.header.frame_id
        self.mapx = cv.CreateImage((self.width, self.height), cv.IPL_DEPTH_32F, 1)
        self.mapy = cv.CreateImage((self.width, self.height), cv.IPL_DEPTH_32F, 1)
        cv.InitUndistortRectifyMap(self.K, self.D, self.R, self.P, self.mapx, self.mapy)

        self.rvec = self.get_rvec()
        self.tvec = self.get_tvec()

    def rectifyImage(self, raw, rectified):
        """
        :param raw:       input image
        :type raw:        :class:`CvMat` or :class:`IplImage`
        :param rectified: rectified output image
        :type rectified:  :class:`CvMat` or :class:`IplImage`

        Applies the rectification specified by camera parameters :math:`K` and and :math:`D` to image `raw` and writes the resulting image `rectified`.
        """

        cv.Remap(raw, rectified, self.mapx, self.mapy)
        
    def rectifyPoint(self, uv_raw):
        """
        :param uv_raw:    pixel coordinates
        :type uv_raw:     (u, v)

        Applies the rectification specified by camera parameters
        :math:`K` and and :math:`D` to point (u, v) and returns the
        pixel coordinates of the rectified point.
        """

        src = mkmat(1, 2, list(uv_raw))
        src = cv.Reshape(src, 2)
        dst = cv.CloneMat(src)
        cv.UndistortPoints(src, dst, self.K, self.D, self.R, self.P)
        return dst[0,0]


    def tfFrame(self):
        """
        Returns the tf frame name - a string - of the 3d points.  This is
        the frame of the :class:`sensor_msgs.msg.CameraInfo` message.
        """
        pass

    def project3dToPixel(self, point):
        """
        :param point:     3D point
        :type point:      (x, y, z)

        Returns the rectified pixel coordinates (u, v) of the 3D point,
        using the camera :math:`P` matrix.
        This is the inverse of :meth:`projectPixelTo3dRay`.
        """
        src = mkmat(4, 1, [point[0], point[1], point[2], 1.0])
        dst = cv.CreateMat(3, 1, cv.CV_64FC1)
        cv.MatMul(self.P, src, dst)
        x = dst[0,0]
        y = dst[1,0]
        w = dst[2,0]
        if w != 0:
            return (x / w, y / w)
        else:
            return (0.0, 0.0)

    def project3dRectToPixelUnrect(self,point):
        ''' project from the left rectified frame onto the unrectified image'''
        src = cv.CreateMat(1,1,cv.CV_64FC3)
        src[0,0] = point
        dst = cv.CreateMat(1,1,cv.CV_64FC2)
        cv.ProjectPoints2(src,self.rvec,self.tvec,self.K,self.D,dst)
        return dst[0,0]

    def project3dToPixelUnrect(self, point):
        """
        :param point:     3D point
        :type point:      (x, y, z)

        Returns the unrectified pixel coordinates (u, v) of the 3D point,
        using opencv's PrjoectPoints2( ,[0;0;0],[0;0;0], :math:`K`, :math:`D`, )
        which is to say, it projects the points according to the Pmatrix
        """
        src = cv.CreateMat(1, 1, cv.CV_64FC3)
        src[0,0] = point
        dst = cv.CreateMat(1, 1, cv.CV_64FC2)
        tvec = cv.CreateMat(3, 1, cv.CV_64FC1) 
        cv.SetZero(tvec)
        rvec = cv.CreateMat(3, 1, cv.CV_64FC1)
        cv.SetZero(rvec)
        cv.ProjectPoints2(src,rvec,tvec,self.K,self.D,dst)
        return dst[0,0]
        

    def projectPixelTo3dRay(self, uv):
        """
        :param uv:        rectified pixel coordinates
        :type uv:         (u, v)

        Returns the unit vector which passes from the camera center to through rectified pixel (u, v),
        using the camera :math:`P` matrix.
        This is the inverse of :meth:`project3dToPixel`.
        """
        x = (uv[0] - self.cx()) / self.fx()
        y = (uv[1] - self.cy()) / self.fy()
        norm = math.sqrt(x*x + y*y + 1)
        x /= norm
        y /= norm
        z = 1.0 / norm
        return (x, y, z)
    
    def projectPixelsTo3dRays(self,U,V):
        """
        :param uv:        rectified pixel coordinates
        :type uv:         (u, v)

        Returns the unit vector which passes from the camera center to through rectified pixel (u, v),
        using the camera :math:`P` matrix.
        This is the inverse of :meth:`project3dToPixel`.
        """
        X = (U - self.cx()) / self.fx()
        Y = (V - self.cy()) / self.fy()
        norm = np.sqrt(X*X +Y*Y + 1)
        X /= norm
        Y /= norm
        Z = 1.0 / norm
        return (X, Y, Z)

    def projectPixelToImagePlane(self,uv,pixelWidth): 
        u = np.array(self.projectPixelTo3dRay(uv))
        v = u/u[2] * self.fx() * float(pixelWidth); 
        return v

    def intrinsicMatrix(self):
        """ Returns :math:`K`, also called camera_matrix in cv docs """
        return self.K
    def distortionCoeffs(self):
        """ Returns :math:`D` """
        return self.D
    def rotationMatrix(self):
        """ Returns :math:`R` which is the matrix unrect_R_rect that transforms unrectified points into rectified frame """
        return self.R
    def projectionMatrix(self):
        """ Returns :math:`P` """
        return self.P

    def cx(self):
        """ Returns x center """
        return self.P[0,2]
    def cy(self):
        """ Returns y center """
        return self.P[1,2]
    def fx(self):
        """ Returns x focal length """
        return self.P[0,0]
    def fy(self):
        """ Returns y focal length """
        return self.P[1,1]

    def get_rvec(self):
        """ return rotation which moves points in left_rect to this unrectified frame
        This is what you give to cv.ProjectPoints2 """
        Rinv = cv.CreateMat(3, 3, cv.CV_64FC1)
        cv.Invert(self.R,Rinv)
        rvec = cv.CreateMat(3, 1, cv.CV_64FC1)
        cv.Rodrigues2(Rinv,rvec)
        return rvec

    def get_tvec(self):
        """ return translation which moves points in left_rect to this unrectified frame
        This is what you give to cv.ProjectPoints2 """
        tvec = cv.CreateMat(3, 1, cv.CV_64FC1)
        tvec[0,0],tvec[1,0],tvec[2,0] = (- self.P[0,3]/-self.P[0,0],0,0)
        return tvec

class StereoCameraModel:
    """
    An idealized stereo camera.
    """
    def __init__(self):
        self.left = PinholeCameraModel()
        self.right = PinholeCameraModel()

    def fromCameraInfo(self, left_msg, right_msg):
        """
        :param left_msg: left camera parameters
        :type left_msg:  sensor_msgs.msg.CameraInfo
        :param right_msg: right camera parameters
        :type right_msg:  sensor_msgs.msg.CameraInfo

        Set the camera parameters from the :class:`sensor_msgs.msg.CameraInfo` messages.
        """
        self.left.fromCameraInfo(left_msg)
        self.right.fromCameraInfo(right_msg)

        # [ Fx, 0,  Cx,  Fx*-Tx ]
        # [ 0,  Fy, Cy,  0      ]
        # [ 0,  0,  1,   0      ]

        if False: #commented by sandy
            fx = self.right.P[0, 0]
            fy = self.right.P[1, 1]
            cx = self.right.P[0, 2]
            cy = self.right.P[1, 2]
            tx = -self.right.P[0, 3] / fx
        
        fx = self.left.P[0, 0]
        fy = self.left.P[1, 1]
        cxl = self.left.P[0, 2]
        cyl = self.left.P[1, 2]
        cxr = self.right.P[0, 2]
        cyr = self.right.P[1, 2]
        tx = -self.right.P[0, 3] / fx #tx is the baseline

        # Q is:
        #    [ 1, 0,  0, -Clx ]
        #    [ 0, 1,  0, -Cy ]
        #    [ 0, 0,  0,  Fx ]
        #    [ 0, 0, 1 / Tx, (Crx-Clx)/Tx ]

        self.Q = cv.CreateMat(4, 4, cv.CV_64FC1)
        cv.SetZero(self.Q)
        self.Q[0, 0] = 1.0
        self.Q[0, 3] = -cxl
        self.Q[1, 1] = 1.0
        self.Q[1, 3] = -cyl
        self.Q[2, 3] = fx
        self.Q[3, 2] = 1 / tx
        self.Q[3, 3] = (cxl-cxr) / tx

    def tfFrame(self):
        """
        Returns the tf frame name - a string - of the 3d points.  This is
        the frame of the :class:`sensor_msgs.msg.CameraInfo` message.  It
        may be used as a source frame in :class:`tf.TransformListener`.
        """

        return self.left.tfFrame()

    def project3dToPixel(self, point):
        """
        :param point:     3D point
        :type point:      (x, y, z)

        Returns the rectified pixel coordinates (u, v) of the 3D point, for each camera, as ((u_left, v_left), (u_right, v_right))
        using the cameras' :math:`P` matrices.
        This is the inverse of :meth:`projectPixelTo3d`.
        """
        l = self.left.project3dToPixel(point)
        r = self.right.project3dToPixel(point)
        return (l, r)

    """def projectDisparityImageTo3D(self, D):
        D = D.copy(); 
        D[np.isnan(D)] = 0; 
        D[D < 0] = 0;
        [X,Y] = np.meshgrid(np.arange(D.shape[1]),np.arange(D.shape[0]));
        XYZ1 = np.vstack([X.flatten(),Y.flatten(),D.flatten()])
        XYZ1 = np.vstack([XYZ1,np.ones(XYZ1.shape[1])])
        src = cvbf.fromarray(XYZ1); 
        dst = cv.CreateMat(4, XYZ1.shape[1], cv.CV_64FC1)
        cv.SetZero(dst)
        cv.MatMul(self.Q, src, dst)
        res = np.array(dst); 
        X = np.reshape(res[0],D.shape); 
        Y = np.reshape(res[1],D.shape); 
        Z = np.reshape(res[2],D.shape); 
        W = np.reshape(res[3],D.shape); 
        X /= W;  
        Y /= W;  
        Z /= W;  
        X[W == 0] = 0; 
        Y[W == 0] = 0; 
        Z[W == 0] = 0; 
        return (X,Y,Z);  
	"""
	
    def projectPixelTo3d(self, left_uv, disparity):
        """
        :param left_uv:        rectified pixel coordinates
        :type left_uv:         (u, v)
        :param disparity:        disparity, in pixels
        :type disparity:         float

        Returns the 3D point (x, y, z) for the given pixel position,
        using the cameras' :math:`P` matrices.
        This is the inverse of :meth:`project3dToPixel`.
        
        Note that a disparity of zero implies that the 3D point is at infinity.
        """
        src = mkmat(4, 1, [left_uv[0], left_uv[1], disparity, 1.0])
        dst = cv.CreateMat(4, 1, cv.CV_64FC1)
        cv.SetZero(dst)
        cv.MatMul(self.Q, src, dst)
        x = dst[0, 0]
        y = dst[1, 0]
        z = dst[2, 0]
        w = dst[3, 0]
        if w != 0:
            return (x / w, y / w, z / w)
        else:
            return (0.0, 0.0, 0.0)


    def leftr_T_rightr(self):
        return kdl.Frame(kdl.Rotation.Identity(),kdl.Vector(-self.right.P[0,3]/self.right.P[0,0],0,0))
        
    def left_T_right(self):
        left_T_leftr = kdl.Frame(cv2kdl_rot(self.left.R).Inverse(),kdl.Vector())
        leftr_T_rightr = self.leftr_T_rightr()
        rightr_T_right = kdl.Frame(cv2kdl_rot(self.right.R),kdl.Vector())
        return left_T_leftr * leftr_T_rightr * rightr_T_right

