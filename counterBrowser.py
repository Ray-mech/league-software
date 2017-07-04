import PyQt5
from PyQt5.QtCore import QUrl 
from PyQt5.QtWidgets import QApplication, QWidget 
from PyQt5.QtWebKitWidgets import QWebView , QWebPage
from PyQt5.QtWebKit import QWebSettings
from PyQt5.QtNetwork import *
import sys
from optparse import OptionParser
 
 
class MyBrowser(QWebPage):
    ''' Settings for the browser.'''
 
    def userAgentForUrl(self, url):
        ''' Returns a User Agent that will be seen by the website. '''
        return "Mozilla/5.0 (Windows NT 6.1) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/41.0.2228.0 Safari/537.36"
 
class Browser(QWebView):
    def __init__(self):
        # QWebView
        self.view = QWebView.__init__(self)
        #self.view.setPage(MyBrowser())
        self.setWindowTitle('Loading...')
        self.titleChanged.connect(self.adjustTitle)
        #super(Browser).connect(self.ui.webView,QtCore.SIGNAL("titleChanged (const QString&)"), self.adjustTitle)
 
    def load(self,url):  
        self.setUrl(QUrl(url)) 
 
    def adjustTitle(self):
        self.setWindowTitle(self.title())
 
    def disableJS(self):
        settings = QWebSettings.globalSettings()
        settings.setAttribute(QWebSettings.JavascriptEnabled, False)
 
class OpenBrowser():
    def makeUrl(self, opponent, myChamp):
        url = "http://lolcounter.com/tips/" + opponent + "/" + myChamp
        print(url)
        return url
        
    def main(self, opponent, myChamp):
        app = QApplication(sys.argv) 
        view = Browser()
        view.show()
        #view.showMaximized()
        view.load(self.makeUrl(opponent, myChamp))
        app.exec_()

web = OpenBrowser()
web.main("rammus","galio")
