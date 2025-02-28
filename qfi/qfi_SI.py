#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import division
from pathlib import Path
from PyQt5.QtGui import QTransform
from PyQt5.QtCore import pyqtSignal, QPointF, Qt
from PyQt5.QtSvg import QGraphicsSvgItem
from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsItem, QFrame
import math

images_path = Path("qfi").resolve()
hand_img_path = f'{images_path / "images" / "asi" / "asi_hand.svg"}'
face_img_path = f'{images_path / "images" / "asi" / "asi_face.svg"}'
case_img_path = f'{images_path / "images" / "asi" / "asi_case.svg"}'

class qfi_SI (QGraphicsView):

    viewUpdate = pyqtSignal()


    def __init__(self,winParent):
        QGraphicsView.__init__(self)

        self.winParent=winParent

        self.viewUpdate.connect(self.update)
        
        self.m_speed = 0

        self.m_scaleX = 0
        self.m_scaleY = 0

        self.m_originalHeight = 240
        self.m_originalWidth = 240

        self.m_originalSiCtr = QPointF(120,120)


        self.m_faceZ = -20
        self.m_handZ = -10
        self.m_caseZ = 10

        self.m_itemHand = None
        self.m_itemFace = None
        self.m_itemCase = None

        self.setStyleSheet("background: transparent; border: none")
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        self.setInteractive(False)
        self.setEnabled(False)


        self.m_scene = QGraphicsScene(self)
        
        self.setScene(self.m_scene)

        self.init()

        

    def init (self):
        self.m_scaleX = self.width() / self.m_originalWidth
        self.m_scaleY = self.height() / self.m_originalHeight


        self.m_itemHand = QGraphicsSvgItem(hand_img_path)
        self.m_itemHand.setCacheMode (QGraphicsItem.NoCache)
        self.m_itemHand.setZValue( self.m_handZ )
        self.m_itemHand.setTransform( QTransform.fromScale( self.m_scaleX, self.m_scaleY ), True )
        self.m_itemHand.setTransformOriginPoint( self.m_originalSiCtr )
        self.m_scene.addItem (self.m_itemHand)

        self.m_itemFace = QGraphicsSvgItem(face_img_path)
        self.m_itemFace.setCacheMode (QGraphicsItem.NoCache)
        self.m_itemFace.setZValue( self.m_faceZ )
        self.m_itemFace.setTransform( QTransform.fromScale( self.m_scaleX, self.m_scaleY ), True )
        self.m_itemFace.setTransformOriginPoint( self.m_originalSiCtr )
        self.m_scene.addItem (self.m_itemFace)

        self.m_itemCase = QGraphicsSvgItem(case_img_path)
        self.m_itemCase.setCacheMode (QGraphicsItem.NoCache)
        self.m_itemCase.setZValue( self.m_caseZ )
        self.m_itemCase.setTransform( QTransform.fromScale( self.m_scaleX, self.m_scaleY ), True )
        self.m_itemCase.setTransformOriginPoint( self.m_originalSiCtr )
        self.m_scene.addItem (self.m_itemCase)

        self.centerOn (self.width()/2, self.height()/2)

        self.updateView()

    def reinit(self):
        if (self.m_scene):
            self.m_scene.clear()
            self.init()


    def update(self):
        self.updateView()


    def setSpeed (self, speed):
        self.m_speed = speed

        if (self.m_speed < 0):
            self.m_speed = 0
        if (self.m_speed > 16):
            self.m_speed = 16


    def resizeEvent (self, event):
        QGraphicsView.resizeEvent (self,event)
        self.reinit()

    def reset (self):
        self.m_itemHand = None
        self.m_itemFace = None
        self.m_itemCase   = None

        self.m_speed =  0.0


    def updateView(self):

        angle = 0

        angle = self.m_speed * 20
        
        self.m_itemHand.setRotation( angle )

        self.m_scene.update()
