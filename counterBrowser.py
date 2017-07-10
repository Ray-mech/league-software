from PyQt5.QtCore import QUrl
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWebKitWidgets import QWebView
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5 import QtCore, QtGui, QtWidgets
import sys

"""生成されたフォームのファイルからフォームをインポート"""
from leaguebrowser_qtui import Ui_Qt_CV_MainWindow

"""おまじない その1"""
class MainWindow(QtWidgets.QMainWindow, Ui_Qt_CV_MainWindow):
    def __init__(self, parent=None):     
        super(MainWindow, self).__init__(parent)
        self.ui = Ui_Qt_CV_MainWindow()
        self.setupUi(self)
        self.url_button.clicked.connect(self.pressed)
        

    def pressed(self):
        self.webView.setUrl(QUrl(self.url_edit.displayText()))
            

if __name__ == '__main__':
     app = QApplication(sys.argv)
     view = MainWindow()
     view.show()
     sys.exit(app.exec_())
