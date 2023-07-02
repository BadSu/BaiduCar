# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'line.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

import os
from PyQt5 import QtCore, QtGui, QtWidgets

logotop = os.path.split(os.path.realpath(__file__))[0]
logotop_x = os.path.join(logotop,"logo.ico")

class Ui_Linewindow(object):
    def setupUi(self, Linewindow):
        Linewindow.setObjectName("Linewindow")
        Linewindow.resize(640, 450)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(logotop_x), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        Linewindow.setWindowIcon(icon)
        self.centralwidget = QtWidgets.QWidget(Linewindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName("gridLayout")
        self.b17 = QtWidgets.QVBoxLayout()
        self.b17.setObjectName("b17")
        self.SXTLine = QtWidgets.QGroupBox(self.centralwidget)
        self.SXTLine.setObjectName("SXTLine")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.SXTLine)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setContentsMargins(78, 0, 78, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.CX = QtWidgets.QLabel(self.SXTLine)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.CX.sizePolicy().hasHeightForWidth())
        self.CX.setSizePolicy(sizePolicy)
        self.CX.setMinimumSize(QtCore.QSize(440, 330))
        self.CX.setText("")
        self.CX.setAlignment(QtCore.Qt.AlignCenter)
        self.CX.setObjectName("CX")
        self.horizontalLayout.addWidget(self.CX)
        self.verticalLayout_3.addLayout(self.horizontalLayout)
        self.b17.addWidget(self.SXTLine)
        self.a19 = QtWidgets.QHBoxLayout()
        self.a19.setContentsMargins(0, -1, 0, -1)
        self.a19.setObjectName("a19")
        self.WCCXAN = QtWidgets.QPushButton(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.WCCXAN.sizePolicy().hasHeightForWidth())
        self.WCCXAN.setSizePolicy(sizePolicy)
        self.WCCXAN.setObjectName("WCCXAN")
        self.a19.addWidget(self.WCCXAN)
        self.b17.addLayout(self.a19)
        self.b17.setStretch(0, 8)
        self.gridLayout.addLayout(self.b17, 0, 0, 1, 1)
        Linewindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(Linewindow)
        QtCore.QMetaObject.connectSlotsByName(Linewindow)

    def retranslateUi(self, Linewindow):
        _translate = QtCore.QCoreApplication.translate
        Linewindow.setWindowTitle(_translate("Linewindow", "line"))
        self.SXTLine.setTitle(_translate("Linewindow", "校准摄像头角度画面"))
        self.WCCXAN.setText(_translate("Linewindow", " 返回"))

