#!/usr/bin/python3

from PyQt4.QtGui import QIcon, QPixmap, QAction, QDialog, QLabel,QTextFormat
from PyQt4.QtCore import SIGNAL,pyqtProperty,QVariant
from PyQt4.QtDesigner import QPyDesignerCustomWidgetPlugin, QExtensionFactory, QPyDesignerTaskMenuExtension, QPyDesignerPropertySheetExtension,QDesignerFormWindowInterface
from qtvcp_widgets.state_label import Lcnc_State_Label
from qtvcp_widgets.qtvcp_icons import Icon
ICON = Icon()

class StateLabelPlugin(QPyDesignerCustomWidgetPlugin):

    def __init__(self, parent=None):
        super(StateLabelPlugin, self).__init__(parent)

        self.initialized = False

    def initialize(self, formEditor):
        if self.initialized:
            return
        # add a custom property editor
#        manager = formEditor.extensionManager()
#        if manager:
#          self.factory = StateLabelPropertySheetExtension(manager)
#          manager.registerExtensions(self.factory,"com.trolltech.Qt.Designer.PropertySheet")

        self.initialized = True

    def isInitialized(self):
        return self.initialized

    def createWidget(self, parent):
        return Lcnc_State_Label(parent)

    def name(self):
        return "Lcnc_State_Label"

    def group(self):
        return "Linuxcnc - Controller"

    def icon(self):
        return QIcon(QPixmap(ICON.get_path('lcnc_state_label')))

    def toolTip(self):
        return ""

    def whatsThis(self):
        return ""

    def isContainer(self):
        return False

    # Returns an XML description of a custom widget instance that describes
    # default values for its properties. Each custom widget created by this
    # plugin will be configured using this description.
    def domXml(self):
        return '<widget class="Lcnc_State_Label" name="lcnc_state_label" />\n'

    def includeFile(self):
        return "qtvcp_widgets.state_label"

#*************************************************************************
class StateLabelPropertySheetExtension(QExtensionFactory):
  def __init__(self, parent = None):

      QExtensionFactory.__init__(self, parent)
      #print 'extension',parent

  def createExtension(self, obj, iid, parent):

      if iid != "com.trolltech.Qt.Designer.PropertySheet":
          return None

      if isinstance(obj, Lcnc_State_Label):
          return StateLabelPropertySheet(obj, parent)

      return None

class StateLabelPropertySheet(QPyDesignerPropertySheetExtension):
    
    def __init__(self, widget, parent):
        QPyDesignerPropertySheetExtension.__init__(self, parent)
        self.widget = widget
        self.formWindow = QDesignerFormWindowInterface.findFormWindow(self.widget)
        print self.formWindow
        self.propertylist=['objectName','geometry','text']
        self.temp_flag = True
        #print dir(self.widget.pyqtConfigure.__sizeof__)
        #print self.widget.pyqtConfigure.__sizeof__()
        for i in Lcnc_State_Label.__dict__:
            #print i
            if 'PyQt4.QtCore.pyqtProperty'  in str(Lcnc_State_Label.__dict__[i]):
                self.propertylist.append(i)
                print i
        #print dir(self.widget)

    def count(self):
        return len(self.propertylist)

    def property(self,index):
        name = self.propertyName(index)
        print 'property index:', index,name
        if 'object' in name:
            return QVariant('default')
        if 'orient' in name:
            return QVariant(False)
        if 'text' == name or 'alt' in name:
            return QVariant(self.widget.text)
        return QVariant(self.widget[str(name)])

    def indexOf(self,name):
        #print 'NAME:',name
        for num,i in enumerate(self.propertylist):
            if i == name: return num
        self.propertylist.append(name)
        print 'not found:',name, num+1
        return num +1

    def setChanged(self, index, value):
        return 

    def isChanged(self, index):
        return False

    def hasReset(self, index):
        return True

    def isAttribute(self, index):
        return False

    def propertyGroup(self, index):
        name = self.propertyName(index)
        if 'geometry' in name:
            return 'QObject'
        if 'objectName' in name:
            return 'QWidget'
        if 'text' in name:
            return 'Text'
        return 'Bool'

    def setProperty(self, index, vvalue):
        prop = self.propertyName(index)
        value = vvalue.toPyObject()
        print 'SET property',prop,index,value
        if 'objectName' in prop:
            self.widget.setObjectName(value)
        if 'geometry' in prop:
            self.widget.setGeometry(value)
        if prop is 'text':
            self.widget.setText(value)
        if 'status' in prop:
            self.do_alt_text_test(prop, index,value)
        self.widget[prop] = value

        return
        if self.formWindow:
            self.formWindow.cursor().setProperty(self.propertyName(index), QVariant(value))
        return

    def getVisible(self, index, data):
        pass

    def isVisible(self, index):
        prop = self.propertyName(index)
        if 'alt_text' in prop:
            return self.temp_flag
        return True

    def propertyName(self, index):
        return self.propertylist[index]

    def do_alt_text_test(self, prop, indexm, value):
        if 'jograte' in prop:
            print 'flag:',value
            self.temp_flag = value


