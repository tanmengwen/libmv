#include <QApplication>
#include <QFileDialog>
#include <QList>
#include <QImage>

#include "../multi_view_stereo_viewer.h"

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);


  MultiViewStereoViewer w;
  QString path = QFileDialog::getOpenFileName(&w,
                                              "Open multi-view dataset",
                                              "/home/paulinus/pro/data",
                                              "MVdata Files (*.txt)");
  QStringList words = path.split("/");
  QString fileName = words.last();
  QStringList fileNameParts = fileName.split("_");
  fileNameParts.pop_back();
  QString baseName = fileNameParts.join("_");
  words.pop_back();
  QString dirName = words.join("/");

  w.setDataset(dirName.toLatin1().data(), baseName.toLatin1().data());

  w.resize(600,400);
  w.show();

  return app.exec();
}
