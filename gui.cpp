//    This file is part of 3DFOREST  www.3dforest.eu
//
//    3DFOREST is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    3DFOREST is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with 3DFOREST.  If not, see <http://www.gnu.org/licenses/>.
//////////////////////////////////////////////////////////////////////
#include "gui.h"

////INPUTDIALOG
InputDialog::InputDialog( QWidget *parent)
: QDialog(parent)
{
  DialogSize(500,300);

  isIC1 = false;
  isIC2 = false;
  isIC3 = false;
  isOC1 = false;
  isOC2 = false;
  isII1 = false;
  isType = false;
//buttons
  buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);

  connect(buttonBox, SIGNAL(accepted()), this, SLOT(ok()));
  connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
  connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

  DialogLayout();
}
void InputDialog::ok()
{
  if(isIC1 == true)
    input_cloud1 = inputCloud1->currentText();
  if(isIC2 == true)
    input_cloud2 = inputCloud2->currentText();
  if(isIC3 == true)
    input_cloud3 = inputCloud3->currentText();
  if(isOC1 == true)
    output_cloud1 = outputCloud1->text();
  if(isOC2 == true)
    output_cloud2 = outputCloud2->text();
  if(isII1 == true)
    int_value1 = intInput->text().toInt();
  if(isType == true)
    output_type = outputType->currentText();
}
void InputDialog::DialogSize(int w = 500, int h = 300)
{
  resize(w, h);
}
void InputDialog::DialogLayout()
{
  // buttons
  buttontLayout = new QHBoxLayout();
  buttontLayout->setGeometry(QRect(0,0,400,50));
  buttontLayout->setAlignment(Qt::AlignRight);
  buttontLayout->setSpacing(10);
  buttontLayout->addWidget(buttonBox );
  //buttontLayout->addWidget(okButton);
  //buttontLayout->addWidget(cancelButton);

  //inputlayout of cloudnames and labels
  InputLayout = new QVBoxLayout();

// input of cloud and main label
  inputareaLayout = new QHBoxLayout();
  inputareaLayout->addLayout(InputLayout);

//mainlayout
  mainLayout = new QVBoxLayout();
  mainLayout->addLayout(inputareaLayout);
  mainLayout->addLayout(buttontLayout);
  setLayout(mainLayout);

}
void InputDialog::set_inputCloud1(QString label, QStringList li)
{
//input cloud
  QLabel *label1 = new QLabel();
  label1->setText(label);
  inputCloud1= new QComboBox();
  inputCloud1->insertItems(0,li);
//layout
  InputLayout->addWidget(label1);
  InputLayout->addWidget(inputCloud1);
  isIC1 =true;
}

void InputDialog::set_inputCloud2(QString label, QStringList li)
{
//input cloud
  QLabel *label2 = new QLabel();
  label2->setText(label);

  inputCloud2 = new QComboBox();
  inputCloud2->insertItems(0,li);
  label2->setBuddy(inputCloud2);
//layout
  InputLayout->addWidget(label2);
  InputLayout->addWidget(inputCloud2);
  isIC2 =true;
}
void InputDialog::set_inputCloud3(QString label, QStringList li)
{
//input cloud
  QLabel *labeli3 = new QLabel();
  labeli3->setText(label);

  inputCloud3 = new QComboBox();
  inputCloud3->insertItems(0,li);
  labeli3->setBuddy(inputCloud3);
//layout
  InputLayout->addWidget(labeli3);
  InputLayout->addWidget(inputCloud3);
  isIC3 =true;
}

void InputDialog::set_description(QString text)
{
  // text about function
  QLabel *label = new QLabel();
  label->setText(text);
  label->setMaximumSize(180,300);
  label->setWordWrap(true);
  label->setMargin(10);
  label->setAlignment(Qt::AlignJustify | Qt::AlignTop);
  //layout
  inputareaLayout->addWidget(label);

}

void InputDialog::set_title(QString title)
{
  setWindowTitle ( title );
}
void InputDialog::set_outputCloud1(QString label, QString example)
{
  //output cloud
  QLabel *labelOC1 = new QLabel();
  labelOC1 ->setText(label);
  outputCloud1 = new QLineEdit;

  output1Bool=true;
  labelOC1 ->setBuddy(outputCloud1);
  connect(outputCloud1,SIGNAL(textChanged(QString)),this,SLOT(validateOutput1(QString)));
  connect(outputCloud1,SIGNAL(textChanged(QString)),this,SLOT(validate(QString)));
  outputCloud1->setText(example);
//layout
  InputLayout->addWidget(labelOC1 );
  InputLayout->addWidget(outputCloud1);
  isOC1 =true;
}
void InputDialog::set_outputCloud2(QString label, QString example)
{
  //output cloud
  QLabel *labelOC2 = new QLabel();
  labelOC2->setText(label);
  outputCloud2 = new QLineEdit;

  output2Bool=true;
  labelOC2 ->setBuddy(outputCloud2);
  connect(outputCloud2,SIGNAL(textChanged(QString)),this,SLOT(validateOutput2(QString)));
  connect(outputCloud2,SIGNAL(textChanged(QString)),this,SLOT(validate(QString)));
  outputCloud2->setText(example);
//layout
  InputLayout->addWidget(labelOC2 );
  InputLayout->addWidget(outputCloud2);
  isOC2 =true;
}
void InputDialog::set_inputInt(QString label, QString x)
{
  QLabel *labelint = new QLabel();
  labelint->setText(label);
  intInput = new QLineEdit;
  intInput->setText(x);
  intInputBool=true;
  intInput->setCursorPosition(0);
  labelint->setBuddy(intInput);
  //resolution
  InputLayout->addWidget(labelint);
  InputLayout->addWidget(intInput);

//ACTION
  connect(intInput,SIGNAL(textChanged(QString)),this,SLOT(validateInt(QString)));
  connect(intInput,SIGNAL(textChanged(QString)),this,SLOT(validate(QString)));
  isII1 = true;
}
void InputDialog::set_outputType(QString label, QStringList li)
{
  //input cloud
  QLabel *labelType = new QLabel();
  labelType->setText(label);

  outputType = new QComboBox();
  outputType->insertItems(0,li);
  labelType->setBuddy(outputType);
  outputType->setMaximumWidth(150);
//layout
  InputLayout->addWidget(labelType);
  InputLayout->addWidget(outputType);
  isType = true;
}
void InputDialog::set_stretch()
{
 InputLayout->addStretch();
}
 void InputDialog::set_path(QString path_m)
 {
   path= path_m;
 }
void InputDialog::validateInt(QString text)
{
   if (text.contains(QRegExp("^\\d+")))
  {
    intInputBool= true;
  }
  else
  {
    buttonBox->setEnabled(false);
    intInputBool= false;
  }
}
void InputDialog::validateOutput1(QString text)
{
  if (text.contains(QRegExp("^\\S+$")))
  {
    QString fullName = QString("%1//%2.pcd").arg(path).arg(text);
    QFile file(fullName);
    if(file.exists())
    {
      outputCloud1->setStyleSheet("QLineEdit{background: red;}");
      buttonBox->setEnabled(false);
    }
    else
    {
      outputCloud1->setStyleSheet("QLineEdit{background: white;}");
      output1Bool= true;
    }
  }
  else
  {
    buttonBox->setEnabled(false);
    output1Bool= false;
  }
}
void InputDialog::validateOutput2(QString text)
{
  if (text.contains(QRegExp("^\\S+$")))
  {
    QString fullName = QString("%1//%2.pcd").arg(path).arg(text);
    QFile file(fullName);
    if(file.exists())
    {
      outputCloud2->setStyleSheet("QLineEdit{background: red;}");
      buttonBox->setEnabled(false);
    }
    else
    {
      outputCloud2->setStyleSheet("QLineEdit{background: white;}");
      output2Bool= true;
    }
  }
  else
  {
    buttonBox->setEnabled(false);
    output2Bool= false;
  }
}
void InputDialog::validate(QString text)
{
  if(isOC1 == true && isOC2 == true && isII1 == true)
  {
    if (intInputBool == true  && output1Bool == true && output2Bool==true )
    {
      buttonBox->setEnabled(true);
    }
    else { buttonBox->setEnabled(false);}
  }
  else if(isOC1 == true && isOC2 == true && isII1 == false)
  {
    if (output1Bool == true && output2Bool==true )
    {
      buttonBox->setEnabled(true);
    }
    else { buttonBox->setEnabled(false);}
  }
  else if(isOC1 == true && isOC2 == false && isII1 == true)
  {
    if (output1Bool == true && intInputBool == true )
    {
      buttonBox->setEnabled(true);
    }
    else { buttonBox->setEnabled(false);}
  }
  else if(isOC1 == false && isOC2 == true && isII1 == true)
  {
    if (output2Bool == true && intInputBool == true )
    {
      buttonBox->setEnabled(true);
    }
    else { buttonBox->setEnabled(false);}
  }
  else if(isOC1 == false && isOC2 == false && isII1 == true)
  {
    if (intInputBool == true )
    {
      buttonBox->setEnabled(true);
    }
    else { buttonBox->setEnabled(false);}
  }
  else if(isOC1 == true && isOC2 == false && isII1 == false)
  {
    if (output1Bool == true )
    {
      buttonBox->setEnabled(true);
    }
    else { buttonBox->setEnabled(false);}
  }
  else if(isOC1 == false && isOC2 == true && isII1 == false)
  {
    if (output2Bool == true  )
    {
      buttonBox->setEnabled(true);
    }
    else { buttonBox->setEnabled(false);}
  }
//  else if(isOC1 == false && isOC2 == false && isII1 == false && isIC1 == true)
//  {
//    if (input1Bool == true  )
//    {
//      buttonBox->setEnabled(true);
//    }
//    else { buttonBox->setEnabled(false);}
//  }
}

QString InputDialog::get_inputCloud1()
{
  return input_cloud1;
}
QString InputDialog::get_inputCloud2()
{
  return input_cloud2;
}
QString InputDialog::get_inputCloud3()
{
  return input_cloud3;
}
QString InputDialog::get_outputCloud1()
{
  return output_cloud1;
}
QString InputDialog::get_outputCloud2()
{
  return output_cloud2;
}
int InputDialog::get_intValue()
{
  return int_value1;
}
QString InputDialog::get_outputType()
{
  return output_type;
}
//ExportAttr dialog
ExportAttr::ExportAttr(QWidget *parent)
: QDialog(parent)
{
  DialogSize(500,300);


//buttons
  buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);

  treeLabel = new QLabel();
  treeLabel->setText("Select tree name:");
  inputTrees = new QComboBox();

  fileLabel = new QLabel();
  fileLabel->setText("Enter name of the file you want to save results:");
  outputFile = new QLineEdit;
  outputFile->setText("C:\\tree_atributes.txt");
  outputFile->setReadOnly(true);
  directoryButton = new QPushButton(tr("Browse"));

  connect(buttonBox, SIGNAL(accepted()), this, SLOT(ok()));
  connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
  connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
  connect(directoryButton, SIGNAL(clicked()), this, SLOT(setExistingDirectory()));


  DialogLayout();

  DBH_HT = false;
  DBH_LSR = false;
  Position = false;
  Height = false;
  Length = false;
  Points = false;
  Areaconvex = false;
  Areaconcave = false;
}
void ExportAttr::DialogSize(int w = 500, int h = 300)
{
  resize(w, h);
}
void ExportAttr::DialogLayout()
{
  // buttons
  buttontLayout = new QHBoxLayout();
  buttontLayout->setGeometry(QRect(0,0,400,50));
  buttontLayout->setAlignment(Qt::AlignRight);
  buttontLayout->setSpacing(10);
  buttontLayout->addWidget(buttonBox );
  // tree selection
  treeLayout = new QVBoxLayout();
  treeLayout->addWidget(treeLabel);
  treeLayout->addWidget(inputTrees);
  treeLayout->setSpacing(5);
  //file selection
  fileLayout = new QGridLayout();
  fileLayout->addWidget(fileLabel,1,0,1,2);
  fileLayout->addWidget(outputFile,2,1);
  fileLayout->addWidget(directoryButton,2,2);
  fileLayout->setSpacing(5);

  //inputlayout of cloudnames and labels
  InputLayout = new QVBoxLayout();
  InputLayout->addLayout(treeLayout);
  InputLayout->addLayout(fileLayout);
  InputLayout->addWidget(separatorGroup());
  InputLayout->addWidget(attributesGroup());
  InputLayout->setSpacing(10);
// input of cloud and main label
  inputareaLayout = new QHBoxLayout();
  inputareaLayout->addLayout(InputLayout);

//mainlayout
  mainLayout = new QVBoxLayout();
  mainLayout->addLayout(inputareaLayout);
  mainLayout->addLayout(buttontLayout);
  setLayout(mainLayout);

}
void ExportAttr::validate( QString name)
{

}
QString ExportAttr::get_separator()
{
  return m_separator;
}
QString ExportAttr::get_outputFile()
{
  return outputFile->text();
}
bool ExportAttr::get_DBH_HT()
{
  if(DBH_HT == true)
    return true;
  else
    return false;
}
bool ExportAttr::get_DBH_LSR()
{
  if(DBH_LSR == true)
    return true;
  else
    return false;
}
bool ExportAttr::get_Position()
{
  if(Position == true)
    return true;
  else
    return false;
}
bool ExportAttr::get_Height()
{
  if(Height == true)
    return true;
  else
    return false;
}
bool ExportAttr::get_Length()
{
  if(Length == true)
    return true;
  else
    return false;
}
bool ExportAttr::get_Points()
{
  if(Points == true)
    return true;
  else
    return false;
}
bool ExportAttr::get_areaconvex()
{
  if(Areaconvex == true)
    return true;
  else
    return false;
}
bool ExportAttr::get_areaconcave()
{
  if(Areaconcave == true)
    return true;
  else
    return false;
}
QString ExportAttr::get_treeName()
{
  return inputTrees->currentText();
}
void ExportAttr::ok()
{
  //separator
  if(radio1->isChecked())
    m_separator = (";");
  if(radio2->isChecked())
    m_separator = (" ");
  if(radio3->isChecked())
    m_separator = ("\t");
  if(radio4->isChecked())
    m_separator = sep->text();

    //attributes
  if(CHB_DBH_HT->isChecked())
    DBH_HT = true;
  if(CHB_DBH_LSR->isChecked())
    DBH_LSR = true;
  if(CHB_position->isChecked())
    Position = true;
  if(CHB_height->isChecked())
    Height = true;
  if(CHB_length->isChecked())
    Length = true;
  if(CHB_points->isChecked())
    Points = true;
  if(CHB_areacave->isChecked())
    Areaconcave = true;
  if(CHB_areavex->isChecked())
    Areaconvex = true;
}
QGroupBox *ExportAttr::attributesGroup()
{
  QGroupBox *groupBox = new QGroupBox(tr("Tree attributes"));
  groupBox->setFlat(true);

  CHB_DBH_HT = new QCheckBox(tr("DBH computed by \nRandomized Hough Transform"));
  connect(CHB_DBH_HT, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_DBH_LSR = new QCheckBox(tr("DBH computed by \nLeast Square Regression"));
  connect(CHB_DBH_LSR, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_height = new QCheckBox(tr("Tree height"));
  connect(CHB_height, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_length = new QCheckBox(tr("Tree length"));
  connect(CHB_length, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_position = new QCheckBox(tr("X Y Z coordinate \nof tree position"));
  connect(CHB_position, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_areavex = new QCheckBox(tr("Area of convex hull"));
  connect(CHB_areavex, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_areacave = new QCheckBox(tr("Area of concave hull"));
  connect(CHB_areacave, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_points = new QCheckBox(tr("Point number"));
  connect(CHB_points, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_all = new QCheckBox(tr("All attributes"));
  connect(CHB_all, SIGNAL(stateChanged(int)), this, SLOT(all_attributes(int)));

  QGridLayout *box = new QGridLayout();
  box->addWidget(CHB_DBH_HT,1,1);
  box->addWidget(CHB_DBH_LSR,1,2);
  box->addWidget(CHB_height,1,3);
  box->addWidget(CHB_length ,2,1);
  box->addWidget(CHB_position,2,2);
  box->addWidget(CHB_areavex,2,3);
  box->addWidget(CHB_areacave ,3,1);
  box->addWidget(CHB_points,3,2);
  box->addWidget(CHB_all,3,3);
  groupBox->setLayout(box);
  return groupBox;
}
QGroupBox *ExportAttr::separatorGroup()
{
  QGroupBox *groupBox = new QGroupBox(tr("Data separator"));
  groupBox->setFlat(true);

  radio1 = new QRadioButton(tr("Semicolon"));
  radio2 = new QRadioButton(tr("Space"));
  radio2->setChecked(true);
  radio3 = new QRadioButton(tr("Tabulator"));
  radio4 = new QRadioButton(tr("Other:"));
  sep = new QLineEdit(this);
  sep->setFixedWidth(60);
  sep->setReadOnly(true);
  sep->setStyleSheet("QLineEdit{background: lightGrey;}");

  connect(radio4, SIGNAL(toggled(bool)), this, SLOT(other_Separator(bool)));
  QHBoxLayout *hbox = new QHBoxLayout;
  hbox->addWidget(radio1);
  hbox->addWidget(radio2);
  hbox->addWidget(radio3);
  hbox->addWidget(radio4);
  hbox->addWidget(sep);
  hbox->setSpacing(10);
  hbox->addStretch(1);
  groupBox->setLayout(hbox);
  return groupBox;
}
void ExportAttr::set_description(QString text)
{
  // text about function
  QLabel *label = new QLabel();
  label->setText(text);
  //label->setMaximumSize(180,300);
  label->setMinimumWidth(160);
  label->setWordWrap(true);
  label->setMargin(10);
  label->setAlignment(Qt::AlignJustify | Qt::AlignTop);
  //layout
  inputareaLayout->addWidget(label);

}
void ExportAttr::set_trees(QStringList li)
{
  inputTrees->insertItems(0,li);
}
void ExportAttr::setExistingDirectory()
{
  QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), "", tr("Text file (*.txt)"));
  outputFile->setText(fileName);
}
void ExportAttr::other_Separator(bool checked)
{
  if(checked == true)
  {
    sep->setReadOnly(false);
    sep->setStyleSheet("QLineEdit{background: white;}");
  }

  else
  {
    sep->setReadOnly(true);
    sep->setStyleSheet("QLineEdit{background: lightGrey;}");
    sep->setText("");
  }
}
void ExportAttr::all_attributes(int checked)
{
  if(checked == 2)
  {
    CHB_DBH_HT->setChecked(true);
    CHB_DBH_LSR->setChecked(true);
    CHB_height->setChecked(true);
    CHB_length->setChecked(true);
    CHB_position->setChecked(true);
    CHB_areavex->setChecked(true);
    CHB_areacave->setChecked(true);
    CHB_points->setChecked(true);
  }
}
void ExportAttr::all_attr(int checked)
{
  if(checked == 0)
  {
    CHB_all->setChecked(false);
  }
}

//import Project
ProjImport::ProjImport(QWidget *parent)
     : QWizard(parent)
{
  //Q_INIT_RESOURCE(3dforest);
  setPixmap(QWizard::LogoPixmap, QPixmap(":/images/logo.png"));
  setPixmap(QWizard::BannerPixmap,QPixmap(":/images/banner.png"));
  setPixmap(QWizard::WatermarkPixmap, QPixmap(":/images/strom.png"));
  setWizardStyle(QWizard::ModernStyle);

  setPage(Page_Intro, new IntroPage);
  setPage(Page_newProject, new NewProjectPage);
  setPage(Page_transform, new TransformPage);
  setPage(Page_import, new ImportPage);
  setPage(Page_final, new FinalPage);

  setStartId(Page_Intro);
  setWindowTitle(tr("Project manager"));
}

IntroPage::IntroPage(QWidget *parent)
     : QWizardPage(parent)
{
  //Q_INIT_RESOURCE(3dforest);
  setTitle(tr("Welcome in project manager"));
  setSubTitle(tr("Please specify if you want create new project or import existing one."));
  setPixmap(QWizard::WatermarkPixmap, QPixmap(":/images/strom.png"));

  topLabel = new QLabel(tr("This wizard will guide you thru importing of projects from different location or by creating new project."
                              "Please select if you want to import existing project or if you want to create a new one. "));
  topLabel->setWordWrap(true);

  newProjectButton = new QRadioButton(tr("&Create new project"));
  importProjectButton = new QRadioButton(tr("&Import existing project"));
  importProjectButton->setChecked(true);

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(topLabel);
  layout->addWidget(newProjectButton);
  layout->addWidget(importProjectButton);
  setLayout(layout);
 }
int IntroPage::nextId() const
 {
     if (newProjectButton->isChecked()) {
         return ProjImport::Page_newProject;
     } else {
         return ProjImport::Page_import;
     }
 }

NewProjectPage::NewProjectPage(QWidget *parent)
     : QWizardPage(parent)
{
  //Q_INIT_RESOURCE(3dforest);
  setTitle(tr("New project"));
  setSubTitle(tr("Details of the new project."));
  setPixmap(QWizard::WatermarkPixmap, QPixmap(":/images/strom.png"));

  topLabel = new QLabel(tr("Please select name and path of the new project. If any field is red, means that on given path exist folder with the same name."
                           " In that case please change name of the project or path to the project folder."));
  topLabel->setWordWrap(true);

  nameLabel = new QLabel(tr("Project name name (please do not use spaces in name):"));
  nameEdit = new QLineEdit;
  nameLabel->setBuddy(nameEdit);
  registerField("projectName*", nameEdit);


  pathLabel = new QLabel(tr("Path to project folder:"));
  pathEdit = new QLineEdit;
  pathEdit->setText("C:\\");
  pathEdit->setReadOnly(true);
  pathLabel->setBuddy(pathEdit);
  registerField("projectPath*", pathEdit);

  directoryButton = new QPushButton(QPixmap(":/images/browse.png"),("Browse"));

  QHBoxLayout *layoutPath = new QHBoxLayout;
  layoutPath->addWidget(pathEdit);
  layoutPath->addWidget(directoryButton);

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(topLabel);
  layout->addWidget(nameLabel);
  layout->addWidget(nameEdit);
  layout->addWidget(pathLabel);
  layout->addLayout(layoutPath);
  setLayout(layout);

  connect(nameEdit, SIGNAL(textChanged(QString)),this,SLOT(nameCheck(QString)));
  connect(pathEdit, SIGNAL(textChanged(QString)),this,SLOT(pathCheck(QString)));
  connect(directoryButton, SIGNAL(clicked()), this, SLOT(setExistingDirectory()));
}
void NewProjectPage::setExistingDirectory()
{
    QFileDialog::Options options = QFileDialog::DontResolveSymlinks | QFileDialog::ShowDirsOnly;
    directory = QFileDialog::getExistingDirectory(this, tr("Select directory for project"),tr("") , options);
    pathEdit->setText(directory);
}
void NewProjectPage::nameCheck(QString name)
{
  bool used=false;
  QString path = QString("%1\\%2").arg(field("projectPath").toString()).arg(field("projectName").toString());
  QDir myDir(path);

  if(myDir.exists())
    used=true;

  if(used == false && name.contains(QRegExp("^[a-zA-Z0-9_-]*$")))
  {
    nameEdit->setStyleSheet("QLineEdit{background: white;}");
    wizard()->button(QWizard::NextButton)->setEnabled(true);
  }
  else
  {
    nameEdit->setStyleSheet("QLineEdit{background: red;}");
    wizard()->button(QWizard::NextButton)->setEnabled(false);
  }
}
void NewProjectPage::pathCheck(QString name)
{
  bool used=false;
  QString path = QString("%1\\%2").arg(field("projectPath").toString()).arg(field("projectName").toString());
  QDir myDir(path);

  if(myDir.exists())
    used=true;

  if(used == false)
  {
    nameEdit->setStyleSheet("QLineEdit{background: white;}");
    wizard()->button(QWizard::NextButton)->setEnabled(true);
  }
  else
  {
    nameEdit->setStyleSheet("QLineEdit{background: red;}");
    wizard()->button(QWizard::NextButton)->setEnabled(false);
  }
}
int NewProjectPage::nextId() const
{
  // create dir in path;
  QString pathDir = QString("%1\\%2").arg(field("projectPath").toString()).arg(field("projectName").toString());
  QDir myDir(pathDir);

  if(!myDir.exists())
    myDir.mkpath(".");

  return ProjImport::Page_transform;
}

TransformPage::TransformPage(QWidget *parent)
     : QWizardPage(parent)
{
  openProjects();
  //Q_INIT_RESOURCE(3dforest);
  setTitle(tr("New project"));
  setSubTitle(tr("Details of the new project - Transformation matrix"));
  setPixmap(QWizard::WatermarkPixmap, QPixmap(":/images/transform.png"));

  topLabel = new QLabel(tr("Please choose transformation matrix or create new. Tranformation matrix serve as a"));
  topLabel->setWordWrap(true);

  selectButton = new QRadioButton(tr("Choose transform matrix"));
  newButton = new QRadioButton(tr("Create new transform matrix"));
  selectButton->setChecked(true);


  nameLabel = new QLabel(tr("Choose existing transformation:"));
  transformList = new QComboBox;
  transformList->insertItems(0,names);
  nameLabel->setBuddy(transformList);


  projLabel = new QLabel(tr("Name of new transformation matrix (without spaces):"));
  projEdit = new QLineEdit;
  projLabel->setBuddy(projEdit);
  projEdit->setEnabled(false);
  registerField("projName*", projEdit);
  projEdit->setText(names.at(0));

  projXLabel = new QLabel(tr("Enter value for X coordinate:"));
  projXEdit = new QLineEdit;
  projXLabel->setBuddy(projXEdit);
  projXEdit->setEnabled(false);
  registerField("projX*", projXEdit);
  projXEdit->setText(x.at(0));

  projYLabel = new QLabel(tr("Enter value for Y coordinate:"));
  projYEdit = new QLineEdit;
  projYLabel->setBuddy(projYEdit);
  projYEdit->setEnabled(false);
  registerField("projY*", projYEdit);
  projYEdit->setText(y.at(0));

  projZLabel = new QLabel(tr("Enter value for Z coordinate:"));
  projZEdit = new QLineEdit;
  projZLabel->setBuddy(projZEdit);
  projZEdit->setEnabled(false);
  registerField("projZ*", projZEdit);
  projZEdit->setText(z.at(0));


  QGridLayout *layoutgrid = new QGridLayout;
  layoutgrid->setColumnMinimumWidth(1,20);
  layoutgrid->setColumnMinimumWidth(2,80);
  layoutgrid->addWidget(selectButton,1,1,1,2);
  layoutgrid->addWidget(nameLabel,2,2);
  layoutgrid->addWidget(transformList,3,2);

  layoutgrid->addWidget(newButton,4,1,1,2);
  layoutgrid->addWidget(projLabel,5,2);
  layoutgrid->addWidget(projEdit,6,2);
  layoutgrid->addWidget(projXLabel,7,2);
  layoutgrid->addWidget(projXEdit,8,2);
  layoutgrid->addWidget(projYLabel,9,2);
  layoutgrid->addWidget(projYEdit,10,2);
  layoutgrid->addWidget(projZLabel,11,2);
  layoutgrid->addWidget(projZEdit,12,2);


  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(topLabel);
  layout->addLayout(layoutgrid);
  setLayout(layout);

  connect(selectButton, SIGNAL(toggled(bool)), this, SLOT(selectProj(bool)));
  connect(newButton, SIGNAL(toggled(bool)), this, SLOT(newProj(bool)));
  connect(transformList, SIGNAL(currentIndexChanged (int)), this, SLOT(selected(int)));
  connect(projXEdit,SIGNAL(textChanged(QString)),this,SLOT(numberXCheck(QString)));
  connect(projYEdit,SIGNAL(textChanged(QString)),this,SLOT(numberYCheck(QString)));
  connect(projZEdit,SIGNAL(textChanged(QString)),this,SLOT(numberZCheck(QString)));
}

void TransformPage::openProjects()
{
  QFile fileproj ("projects.txt");
  fileproj.open(QIODevice::ReadOnly | QIODevice::Text);
  QTextStream in(&fileproj);
  names << "select projection matrix";
  x << "";
  y << "";
  z << "";
  while(!in.atEnd())
  {
    QString line = in.readLine();
    QStringList coords = line.split(" ");
    names << coords.at(0);
    x << coords.at(1);
    y << coords.at(2);
    z << coords.at(3);
  }
  fileproj.close();
}
int TransformPage::nextId() const
{
  if(newButton->isChecked())
  {
    // pridat zapis do projects
    QFile fileproj ("projects.txt");
    fileproj.open(QIODevice::Append | QIODevice::Text);
    QTextStream out(&fileproj);
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(0);
    out <<"\n"<< field("projName").toString() <<" "<< field("projX").toString() << " " << field("projY").toString()<< " " << field("projZ").toString() ;
    fileproj.close();
  }

  // create proj file
  QString pathDir = QString("%1\\%2").arg(field("projectPath").toString()).arg(field("projectName").toString());
  QString fileN =QString("%1\\%2.3df").arg(pathDir).arg(field("projectName").toString());
  QFile file (fileN);
  file.open(QIODevice::WriteOnly);
  // write header of proj file
  QTextStream out(&file);
  out.setRealNumberNotation(QTextStream::FixedNotation);
  out.setRealNumberPrecision(0);
  out << field("projectName").toString() <<" "<< field("projX").toString() << " " << field("projY").toString()<< " " << field("projZ").toString()<< " "<<  pathDir <<"\n" ;
  file.close();

  return ProjImport::Page_final;
}
void TransformPage::selectProj(bool check)
{
  projEdit->setEnabled(false);
  projXEdit->setEnabled(false);
  projYEdit->setEnabled(false);
  projZEdit->setEnabled(false);
  transformList->setEnabled(true);
  projEdit->setStyleSheet("QLineEdit{background: white;}");
  disconnect(projEdit,SIGNAL(textChanged(QString)),this,SLOT(nameCheck(QString)));
  connect(transformList, SIGNAL(currentIndexChanged (int)), this, SLOT(selected(int)));
}
void TransformPage::newProj(bool check)
{
  disconnect(transformList, SIGNAL(currentIndexChanged (int)), this, SLOT(selected(int)));
  connect(projEdit,SIGNAL(textChanged(QString)),this,SLOT(nameCheck(QString)));
  transformList->setCurrentIndex (0);
  transformList->setEnabled(false);
  projEdit->setEnabled(true);
  projXEdit->setEnabled(true);
  projYEdit->setEnabled(true);
  projZEdit->setEnabled(true);
  projEdit->setText("");
  wizard()->button(QWizard::NextButton)->setEnabled(false);
  projEdit->setStyleSheet("QLineEdit{background: red;}");
}
void TransformPage::selected(int index)
{
  disconnect(projEdit,SIGNAL(textChanged(QString)),this,SLOT(nameCheck(QString)));
  connect(transformList, SIGNAL(currentIndexChanged (int)), this, SLOT(selected(int)));
  projEdit->setText(names.at(index));
  projXEdit->setText(x.at(index));
  projYEdit->setText(y.at(index));
  projZEdit->setText(z.at(index));

}
void TransformPage::nameCheck(QString name)
{
  bool used = false;
  for(int i = 0; i < names.size(); i++)
  {
    if(name == names.at(i))
      used = true;
  }

  if(used == true || name.contains(QRegExp("^\\s+$")))
  {
    projEdit->setStyleSheet("QLineEdit{background: red;}");
  wizard()->button(QWizard::NextButton)->setEnabled(false);
  }

  else
  {
    projEdit->setStyleSheet("QLineEdit{background: white;}");
    wizard()->button(QWizard::NextButton)->setEnabled(true);
  }
}
void TransformPage::numberXCheck(QString name)
{
  if (name.contains(QRegExp("^\\-?\\d+$")))
  {
   projXEdit->setStyleSheet("QLineEdit{background: white;}");
   wizard()->button(QWizard::NextButton)->setEnabled(true);
  }
  else
  {
    projXEdit->setStyleSheet("QLineEdit{background: red;}");
    wizard()->button(QWizard::NextButton)->setEnabled(false);
  }
}
void TransformPage::numberYCheck(QString name)
{
  if (name.contains(QRegExp("^\\-?\\d+$")))
  {
    projYEdit->setStyleSheet("QLineEdit{background: white;}");
    wizard()->button(QWizard::NextButton)->setEnabled(true);
  }
  else
  {
    projYEdit->setStyleSheet("QLineEdit{background: red;}");
    wizard()->button(QWizard::NextButton)->setEnabled(false);
  }
}
void TransformPage::numberZCheck(QString name)
{
  if (name.contains(QRegExp("^\\-?\\d+$")))
  {
    projZEdit->setStyleSheet("QLineEdit{background: white;}");
    wizard()->button(QWizard::NextButton)->setEnabled(true);
  }
  else
  {
    projZEdit->setStyleSheet("QLineEdit{background: red;}");
    wizard()->button(QWizard::NextButton)->setEnabled(false);
  }
}
ImportPage::ImportPage(QWidget *parent)
     : QWizardPage(parent)
{
  //Q_INIT_RESOURCE(3dforest);
  setTitle(tr("Import project"));
  setSubTitle(tr("setup of project"));
  setPixmap(QWizard::WatermarkPixmap, QPixmap(":/images/strom.png"));

  topLabel = new QLabel(tr("Please choose project file that you want to import into new location. Location will be created in new place and all clouds will be moved into new folder."));
  topLabel->setWordWrap(true);

  oldLabel = new QLabel(tr("Path to old project file:"));
  oldpathEdit = new QLineEdit;
  oldpathEdit->setText("");
  oldpathEdit->setReadOnly(true);
  oldLabel->setBuddy(oldpathEdit);
  registerField("oldprojectPath*", oldpathEdit);

  oldButton = new QPushButton(QPixmap(":/images/browse.png"),("Browse"));

  projLabel = new QLabel(tr("Name of new project"));
  projEdit = new QLineEdit;
  projLabel->setBuddy(projEdit);
  registerField("newprojectname*", projEdit);

  newLabel = new QLabel(tr("Path to new project folder:"));
  newpathEdit = new QLineEdit;
  newpathEdit->setText("C:\\");
  newpathEdit->setReadOnly(true);
  newLabel->setBuddy(newpathEdit);
  registerField("newprojectPath*", newpathEdit);

  newButton = new QPushButton(QPixmap(":/images/browse.png"),("Browse"));
  removeCheckBox = new QCheckBox ( tr("remove old project?"),this);

  QGridLayout *layoutgrid = new QGridLayout;
  layoutgrid->setColumnMinimumWidth(1,20);
  layoutgrid->setColumnMinimumWidth(2,80);
  layoutgrid->addWidget(oldLabel,1,2);
  layoutgrid->addWidget(oldpathEdit,2,2);
  layoutgrid->addWidget(oldButton,2,3);
  layoutgrid->addWidget(projLabel,3,2);
  layoutgrid->addWidget(projEdit,4,2);
  layoutgrid->addWidget(newLabel,5,2);
  layoutgrid->addWidget(newpathEdit,6,2);
  layoutgrid->addWidget(newButton,6,3);
  layoutgrid->addWidget(removeCheckBox,7,2);

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(topLabel);
  layout->addLayout(layoutgrid);
  setLayout(layout);

  connect(newButton, SIGNAL(clicked()), this, SLOT(setNewDirectory()));
  connect(oldButton, SIGNAL(clicked()), this, SLOT(setOldFile()));
  connect(projEdit, SIGNAL(textChanged(QString)),this,SLOT(nameCheck(QString)));
  connect(newpathEdit, SIGNAL(textChanged(QString)),this,SLOT(pathCheck(QString)));
}
void ImportPage::setOldFile()
{
  QString file = QFileDialog::getOpenFileName(this,tr("open file"),"",tr("files (*.3df)"));
  oldpathEdit->setText(file);
}
void ImportPage::setNewDirectory()
{
  QFileDialog::Options options = QFileDialog::DontResolveSymlinks | QFileDialog::ShowDirsOnly;
  QString directory = QFileDialog::getExistingDirectory(this, tr("Select directory for project"),tr("") , options);
  newpathEdit->setText(directory);
}
void ImportPage::nameCheck(QString name)
{
  bool used=false;
  QString path = QString("%1\\%2").arg(field("newprojectPath").toString()).arg(field("newprojectname").toString());
  QDir myDir(path);

  if(myDir.exists())
    used=true;

  if(used == false && name.contains(QRegExp("^[a-zA-Z0-9_-]*$")))
  {
    projEdit->setStyleSheet("QLineEdit{background: white;}");
    wizard()->button(QWizard::NextButton)->setEnabled(true);
  }
  else
  {
    projEdit->setStyleSheet("QLineEdit{background: red;}");
    wizard()->button(QWizard::NextButton)->setEnabled(false);
  }
}
void ImportPage::pathCheck(QString name)
{
  bool used=false;
  QString path = QString("%1\\%2").arg(field("newprojectPath").toString()).arg(field("newprojectname").toString());
  QDir myDir(path);

  if(myDir.exists())
    used=true;

  if(used == false)
  {
    projEdit->setStyleSheet("QLineEdit{background: white;}");
    wizard()->button(QWizard::NextButton)->setEnabled(true);
  }
  else
  {
    projEdit->setStyleSheet("QLineEdit{background: red;}");
    wizard()->button(QWizard::NextButton)->setEnabled(false);
  }
}

int ImportPage::nextId() const
{
  // open new project file
  QString pathDir = QString("%1\\%2").arg(field("newprojectPath").toString()).arg(field("newprojectname").toString());
  QDir myDir(pathDir);
  if(!myDir.exists())
    myDir.mkpath(".");

  QString fileN =QString("%1\\%2.3df").arg(pathDir).arg(field("newprojectname").toString());
  QFile filenew (fileN);
  filenew.open(QIODevice::WriteOnly| QIODevice::Text);
  QTextStream out(&filenew);

  // open old project file
  QFile fileold (field("oldprojectPath").toString());
  fileold.open(QIODevice::ReadOnly | QIODevice::Text);
  QTextStream in(&fileold);
  QFileInfo fold (fileold);
  QString fileold_path = fold.canonicalPath();

  bool first_line = true;
  while(!in.atEnd())
  {
    QString lines = in.readLine();
    QStringList coords = lines.split(" ");
//READ FIRST LINE

    if(first_line == true)
    {
      if(coords.size() == 5)
      {
        out << field("newprojectname").toString() << " " << coords.at(1) << " " << coords.at(2)<< " " << coords.at(3)<< " " << pathDir<< "\n";
      }
      else
      {

        out << field("newprojectname").toString() << " " << coords.at(1) << " " << coords.at(2)<< " " << "0"<< " " << pathDir<< "\n";
      }

      first_line = false;
    }
//READ REST OF FILE
    else
    {
      if(coords.size() == 5) //type, path, r,g,b
      {
        QStringList apth = coords.at(1).split("\\");
        QString new_file = QString("%1\\%2").arg(pathDir).arg(apth.at(apth.size()-1));
        QString old_file = QString("%1\\%2").arg(fileold_path).arg(apth.at(apth.size()-1));

        // zkopirovat soubor
        QFile::copy(old_file, new_file);
        //ulozit do proj.3df
        out << coords.at(0) << " " << new_file << " " << coords.at(2)<< " " << coords.at(3)<< " " << coords.at(4)<< "\n";
      }
      else //type, path
      {
        QStringList apth = coords.at(1).split("\\");
        QString new_file = QString("%1\\%2").arg(pathDir).arg(apth.at(apth.size()-1));
        QString old_file = QString("%1\\%2").arg(fileold_path).arg(apth.at(apth.size()-1));
        // zkopirovat soubor
        QFile::copy(old_file, new_file);
        //ulozit do proj.3df
        out << coords.at(0) << " " << new_file <<"\n";
      }
    }
  }
  filenew.close(); // new file close
  fileold.close(); // old file close


  if(removeCheckBox->checkState () == 2)
  {
    //remove old projectQFileInfo f(field("oldprojectPath").toString());
    QFileInfo f(field("oldprojectPath").toString());
    QDir dir(f.canonicalPath ());
    if (dir.exists())
    {
      Q_FOREACH(QFileInfo info, dir.entryInfoList(QDir::NoDotAndDotDot | QDir::System | QDir::Hidden  | QDir::AllDirs | QDir::Files, QDir::DirsFirst))
      {
       QFile::remove(info.absoluteFilePath());
      }
      dir.rmdir(f.canonicalPath());
    }
  }
  return ProjImport::Page_final;
}
FinalPage::FinalPage(QWidget *parent)
     : QWizardPage(parent)
{
  //Q_INIT_RESOURCE(3dforest);
  setTitle(tr("Welcome in project manager"));
  setSubTitle(tr("final conclulsion."));
  setPixmap(QWizard::WatermarkPixmap, QPixmap(":/images/strom.png"));

  topLabel = new QLabel(tr("Your project is created and ready to use!"));
  topLabel->setWordWrap(true);


  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(topLabel);
  setLayout(layout);
 }
int FinalPage::nextId() const
{
  // open project file

  return -1;
}

////MYTREE
MyTree::MyTree(QWidget *parent)
  : QTreeWidget(parent)
{
    setContextMenuPolicy(Qt::CustomContextMenu);
    setColumnCount(2);
    //resizeColumnToContents(1);
    header()->resizeSection(0, 50);
    QStringList q;
    q << QString (" visible") ;
    q << QString ("     name");
    setHeaderLabels(q);
    header()->resizeSection(1, 120);

    //SLOTS
    connect(this, SIGNAL(customContextMenuRequested(const QPoint&)),this, SLOT(showContextMenu(const QPoint&)));
    connect(this, SIGNAL(itemChanged(QTreeWidgetItem*,int)), this, SLOT(onItemChange(QTreeWidgetItem*,int)));
}
void MyTree::itemdelete(QString name)
{
  QTreeWidgetItemIterator it(this);
  while (*it)
  {
    if( (*it)->text(1) == name)
    {
      QTreeWidgetItem *currentItem = (*it);
      delete currentItem;
    }
    ++it;
  }
}
void MyTree::showContextMenu(const QPoint &pos)
{
  QMenu *menu = new QMenu;

  QModelIndex index = this->currentIndex();
  if(!index.isValid())
    return;

  QString it = this->model()->data(this->model()->index(index.row(), 1),0).toString();

  QAction *deleteACT = new QAction("delete",menu);
  menu->addAction(deleteACT);

  QAction *colorACT = new QAction("color",menu);
  menu->addAction(colorACT);

  QAction *colorFieldACT = new QAction("Color by field",menu);
  menu->addAction(colorFieldACT);

  QAction *PsizeACT = new QAction("Point size",menu);
  menu->addAction(PsizeACT);

  QAction *allONACT = new QAction("all Clouds ON",menu);
  menu->addAction(allONACT);
  connect(allONACT, SIGNAL(triggered()), this, SLOT(allON()));

  QAction *allOFFACT = new QAction("all Clouds OFF",menu);
  menu->addAction(allOFFACT);
  connect(allOFFACT, SIGNAL(triggered()), this, SLOT(allOFF()));

  QSignalMapper *signalMapper = new QSignalMapper(this);
  connect(deleteACT, SIGNAL(triggered()), signalMapper, SLOT(map()));
  signalMapper->setMapping(deleteACT, it);
  connect(signalMapper, SIGNAL(mapped(QString)), this, SLOT(onDeleteItem(QString)));

  QSignalMapper *signalMapperC = new QSignalMapper(this);
  connect(colorACT, SIGNAL(triggered()), signalMapperC, SLOT(map()));
  signalMapperC->setMapping(colorACT, it);
  connect(signalMapperC, SIGNAL(mapped(QString)), this, SLOT(onColor(QString)));

  QSignalMapper *signalMapperCF = new QSignalMapper(this);
  connect(colorFieldACT, SIGNAL(triggered()), signalMapperCF, SLOT(map()));
  signalMapperCF->setMapping(colorFieldACT, it);
  connect(signalMapperCF, SIGNAL(mapped(QString)), this, SLOT(onColorField(QString)));

  QSignalMapper *signalMapperPsize = new QSignalMapper(this);
  connect(PsizeACT, SIGNAL(triggered()), signalMapperPsize, SLOT(map()));
  signalMapperPsize->setMapping(PsizeACT, it);
  connect(signalMapperPsize, SIGNAL(mapped(QString)), this, SLOT(onPsize(QString)));

  menu->exec(QCursor::pos());
}
void MyTree::onItemChange(QTreeWidgetItem *item,int i)
{
  name = item->text(1);

  if(!item->checkState(0))
  {
    emit checkedON(name);
  }
 if(item->checkState(0))
  {
    emit checkedOFF(name);
  }
}
void MyTree::onDeleteItem(QString name)
{
  QMessageBox *msgBox =  new QMessageBox(this);
	msgBox->setText("DELETE");
	QString a = QString("DO YOU WANT TO DELETE CLOUD -- %1 -- FROM PROJECT?").arg(name);
	msgBox->setInformativeText(a);
	msgBox->setStandardButtons(QMessageBox::Yes | QMessageBox::No);
	msgBox->setDefaultButton(QMessageBox::Yes);

	if(msgBox->exec() == QMessageBox::Yes)
  {
    itemdelete(name);
    emit deleteItem(name);
  }
}
void MyTree::onColor(QString name)
{
  emit colorItem(name);
}
void MyTree::onColorField(QString name)
{
  emit colorItemField(name);
}
void MyTree::onPsize(QString name)
{
  emit psize(name);
}
void MyTree::cleanAll()
{
  QTreeWidgetItemIterator it(this);
  int index = 0;
  while (*it)
  {
    QTreeWidgetItem *item = (*it);
     if(!item)return;
     int x = this->indexOfTopLevelItem(item);
     if(x >= 0 && x < this->topLevelItemCount())
     {
       item = this->takeTopLevelItem(x);
       if(item)
         delete item;
     }
  }
}
void MyTree::allON()
{
  QTreeWidgetItemIterator it(this);
  while (*it)
  {
    QTreeWidgetItem *item = (*it);
    name = item->text(1);
    item->setCheckState(0,Qt::Checked);
    emit checkedOFF(name);
    it++;
  }
}
void MyTree::allOFF()
{
  allItemOFF();
}
void MyTree::allItemOFF()
{
  QTreeWidgetItemIterator it(this);
  //int index = 0;
  while (*it)
  {
    QTreeWidgetItem *item = (*it);
    name = item->text(1);
    item->setCheckState(0,Qt::Unchecked);
    emit checkedON(name);
    it++;
  }

}
void MyTree::itemON(QString name)
{
QTreeWidgetItemIterator it(this);
  while (*it)
  {
    QTreeWidgetItem *item = (*it);
    if (name == item->text(1))
    {
      item->setCheckState(0,Qt::Checked);
      emit checkedOFF(name);
    }
    it++;
  }
}

////VISUALIZER
Visualizer::Visualizer(QString name)
{
  PCLVisualizer (name.toUtf8().constData());
}
