# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'GUI.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

import os
from PyQt5 import QtCore, QtGui, QtWidgets

logotop = os.path.split(os.path.realpath(__file__))[0]
logotop_x = os.path.join(logotop,"logo.ico")

class Ui_Mainwindow(object):
    def setupUi(self, Mainwindow):
        Mainwindow.setObjectName("Mainwindow")
        Mainwindow.resize(640, 450)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(logotop_x), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        Mainwindow.setWindowIcon(icon)
        self.centralwidget = QtWidgets.QWidget(Mainwindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName("gridLayout")
        self.a = QtWidgets.QHBoxLayout()
        self.a.setContentsMargins(15, 15, 15, 15)
        self.a.setObjectName("a")
        self.b = QtWidgets.QVBoxLayout()
        self.b.setContentsMargins(0, -1, 5, -1)
        self.b.setObjectName("b")
        self.a1 = QtWidgets.QHBoxLayout()
        self.a1.setContentsMargins(0, 0, 0, -1)
        self.a1.setObjectName("a1")
        self.SYSM = QtWidgets.QTextBrowser(self.centralwidget)
        self.SYSM.setObjectName("SYSM")
        self.a1.addWidget(self.SYSM)
        self.b.addLayout(self.a1)
        self.a2 = QtWidgets.QHBoxLayout()
        self.a2.setContentsMargins(-1, -1, 0, -1)
        self.a2.setObjectName("a2")
        self.logo = QtWidgets.QLabel(self.centralwidget)
        self.logo.setStyleSheet("image: url(:/png/HG.png);")
        self.logo.setText("")
        self.logo.setObjectName("logo")
        self.a2.addWidget(self.logo)
        self.b.addLayout(self.a2)
        self.b.setStretch(0, 4)
        self.b.setStretch(1, 2)
        self.a.addLayout(self.b)
        self.b1 = QtWidgets.QVBoxLayout()
        self.b1.setObjectName("b1")
        self.b5 = QtWidgets.QGroupBox(self.centralwidget)
        self.b5.setObjectName("b5")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.b5)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.b6 = QtWidgets.QVBoxLayout()
        self.b6.setObjectName("b6")
        self.a4 = QtWidgets.QHBoxLayout()
        self.a4.setContentsMargins(20, 10, 20, 10)
        self.a4.setObjectName("a4")
        self.RGBMSAN = QtWidgets.QPushButton(self.b5)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.RGBMSAN.sizePolicy().hasHeightForWidth())
        self.RGBMSAN.setSizePolicy(sizePolicy)
        self.RGBMSAN.setObjectName("RGBMSAN")
        self.a4.addWidget(self.RGBMSAN)
        self.b6.addLayout(self.a4)
        self.a5 = QtWidgets.QHBoxLayout()
        self.a5.setContentsMargins(20, 10, 20, 10)
        self.a5.setObjectName("a5")
        self.HSVMSAN = QtWidgets.QPushButton(self.b5)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.HSVMSAN.sizePolicy().hasHeightForWidth())
        self.HSVMSAN.setSizePolicy(sizePolicy)
        self.HSVMSAN.setObjectName("HSVMSAN")
        self.a5.addWidget(self.HSVMSAN)
        self.b6.addLayout(self.a5)
        self.verticalLayout_2.addLayout(self.b6)
        self.b1.addWidget(self.b5)
        self.b3 = QtWidgets.QGroupBox(self.centralwidget)
        self.b3.setObjectName("b3")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.b3)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.a3 = QtWidgets.QHBoxLayout()
        self.a3.setContentsMargins(0, -1, 0, -1)
        self.a3.setObjectName("a3")
        self.b4 = QtWidgets.QVBoxLayout()
        self.b4.setContentsMargins(20, 40, 20, 40)
        self.b4.setObjectName("b4")
        self.CDXSBAN = QtWidgets.QPushButton(self.b3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.CDXSBAN.sizePolicy().hasHeightForWidth())
        self.CDXSBAN.setSizePolicy(sizePolicy)
        self.CDXSBAN.setObjectName("CDXSBAN")
        self.b4.addWidget(self.CDXSBAN)
        self.a3.addLayout(self.b4)
        self.verticalLayout_3.addLayout(self.a3)
        self.b1.addWidget(self.b3)
        self.a.addLayout(self.b1)
        self.a.setStretch(0, 4)
        self.a.setStretch(1, 3)
        self.gridLayout.addLayout(self.a, 0, 0, 1, 1)
        Mainwindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(Mainwindow)
        QtCore.QMetaObject.connectSlotsByName(Mainwindow)

    def retranslateUi(self, Mainwindow):
        _translate = QtCore.QCoreApplication.translate
        Mainwindow.setWindowTitle(_translate("Mainwindow", "GUI"))
        self.SYSM.setHtml(_translate("Mainwindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'SimSun\'; font-size:10pt; font-weight:600;\">使用说明：</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'SimSun\'; font-size:9pt;\">本程序是用于调节\'颜色阈值&quot;和&quot;识别车道线摄像头角度&quot;的</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'SimSun\'; font-size:10pt; font-weight:600;\">颜色阈值模式：</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'SimSun\'; font-size:9pt;\">通过选择识别颜色模式，进行调节相应的色彩阈值范围，使是摄像头可以正确识别到车道线颜色，共有三个相量，最小值为 0 ，最大值为 255 ，在这数值范围内的颜色会在画面中显示成白色，否则为黑。</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'SimSun\'; font-size:9pt; font-weight:600;\">RGB 模式：</span><span style=\" font-family:\'SimSun\'; font-size:9pt;\">通过对红(R),绿(G),蓝(B)三种颜色通道变化和它们相互之间的叠加来得到各式各样的颜色。</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'SimSun\'; font-size:9pt; font-weight:600;\">HSV 模式：</span><span style=\" font-family:\'SimSun\'; font-size:9pt;\">这个颜色模式的参数分别是色调(H),饱和度(S),明度(V)而得到各种颜色。</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'SimSun\'; font-size:10pt; font-weight:600;\">摄像头角度调节：</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'SimSun\'; font-size:9pt;\">通过摄像头画面中给出的辅助线进行调节摄像头角度，使小车可以正常在车道线中行驶。</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'SimSun\'; font-size:9pt; font-weight:600;\">车道线校准：</span><span style=\" font-family:\'SimSun\'; font-size:9pt;\">通过显示画面辅助线进调节摄像头角度，将画面中黄色车道线调节至辅助线中即可。</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:\'SimSun\'; font-size:9pt;\"><br /></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:\'SimSun\'; font-size:9pt;\"><br /></p></body></html>"))
        self.b5.setTitle(_translate("Mainwindow", "颜色阈值模式"))
        self.RGBMSAN.setText(_translate("Mainwindow", "RGB 模式"))
        self.HSVMSAN.setText(_translate("Mainwindow", "HSV 模式"))
        self.b3.setTitle(_translate("Mainwindow", "摄像头角度调节"))
        self.CDXSBAN.setText(_translate("Mainwindow", "车道线校准"))

import UI.logo_rc
