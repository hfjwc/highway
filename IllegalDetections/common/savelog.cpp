#include "savelog.h"
#include "QTextStream"
#include "qcoreapplication.h"
#include "qdatetime.h"
#include "qdir.h"
#include "qfile.h"
#include "qmutex.h"
#include "qstringlist.h"
#include "qtimer.h"

#define QDATE qPrintable(QDate::currentDate().toString("yyyy_MM_dd"))
#define QDATETIMS qPrintable(QDateTime::currentDateTime().toString("yyyy_MM_dd"))

//日志重定向
void Log(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
    //加锁,防止多线程中qdebug太频繁导致崩溃
    static QMutex mutex;
    QMutexLocker  locker(&mutex);
    QString       content;

    //这里可以根据不同的类型加上不同的头部用于区分
    int     msgType = SaveLog::Instance()->getMsgType();
    bool    flag    = false;
    QString strTime = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz");
    // 2022-05-05 11:19:40,210 utils.py[line:412] INFO:{'message': 'Success', 'status': 0}
    int     line = context.line;
    QString file = context.file;
    if (line > 0)
    {
        content = content + QString("%1 %2[line:%3]").arg(strTime).arg(file).arg(line);
    }
    switch (type)
    {
        case QtDebugMsg:
            if ((msgType & MsgType_Debug) == MsgType_Debug)
            {
                content = QString("%1 DEBUG:%2").arg(content).arg(msg);
            }
            break;
        case QtInfoMsg:
            if ((msgType & MsgType_Info) == MsgType_Info)
            {
                content = QString("%1 INFO:%2").arg(content).arg(msg);
            }
            flag = true;
            break;
        case QtWarningMsg:
            if ((msgType & MsgType_Warning) == MsgType_Warning)
            {
                content = QString("%1 WARNING:%2").arg(content).arg(msg);
            }
            break;
        case QtCriticalMsg:
            if ((msgType & MsgType_Critical) == MsgType_Critical)
            {
                content = QString("%1 ERROR:%2").arg(content).arg(msg);
            }
            break;
        default:
            break;
    }
    SaveLog::Instance()->save(content);
}

QScopedPointer<SaveLog> SaveLog::self;
SaveLog*                SaveLog::Instance()
{
    if (self.isNull())
    {
        static QMutex mutex;
        QMutexLocker  locker(&mutex);
        if (self.isNull())
        {
            self.reset(new SaveLog);
        }
    }
    return self.data();
}

SaveLog::SaveLog(QObject* parent)
    : QObject(parent)
{
    //必须用信号槽形式,不然提示 QSocketNotifier: Socket notifiers cannot be enabled or disabled from another thread
    isRun  = false;
    maxRow = currentRow = 0;
    maxSize             = 128;
    toNet               = false;
    //全局的文件对象,在需要的时候打开而不是每次添加日志都打开
    file = new QFile(this);
    //默认取应用程序根目录
    path = qApp->applicationDirPath();
    //默认取应用程序可执行文件名称
    QString     str  = qApp->applicationFilePath();
    QStringList list = str.split("/");
    name             = list.at(list.count() - 1).split(".").at(0);
    m_fileName       = "";
    //默认所有类型都输出
    msgType = MsgType(MsgType_Debug | MsgType_Info | MsgType_Warning | MsgType_Critical | MsgType_Fatal);
}

SaveLog::~SaveLog()
{
    file->close();
}

void SaveLog::openFile(const QString& fileName)
{
    //当文件名改变时才新建和打开文件而不是每次都打开文件(效率极低)或者一开始打开文件
    if (this->m_fileName != fileName)
    {
        this->m_fileName = fileName;
        //先关闭之前的
        if (file->isOpen())
        {
            file->close();
        }
        //重新设置新的日志文件
        file->setFileName(fileName);
        //以 Append 追加的形式打开
        file->open(QIODevice::WriteOnly | QIODevice::Append | QFile::Text);
    }
}

MsgType SaveLog::getMsgType()
{
    return this->msgType;
}

//安装日志钩子,输出调试信息到文件,便于调试
void SaveLog::start()
{
    if (isRun)
    {
        return;
    }
    isRun = true;
    qInstallMessageHandler(Log);
}

//卸载日志钩子
void SaveLog::stop()
{
    if (!isRun)
    {
        return;
    }

    isRun = false;
    this->clear();
    qInstallMessageHandler(0);
}

void SaveLog::clear()
{
    currentRow = 0;
    m_fileName.clear();
    if (file->isOpen())
    {
        file->close();
    }
}

void SaveLog::save(const QString& content)
{
    //目录不存在则先新建目录
    QDir dir(path);
    if (!dir.exists())
    {
        dir.mkdir(path);
    }
    //日志存储规则有多种策略 优先级 行数>大小>日期
    // 1: 设置了最大行数限制则按照行数限制来
    // 2: 设置了大小则按照大小来控制日志文件
    // 3: 都没有设置都存储到日期命名的文件,只有当日期变化了才会切换到新的日志文件
    bool needOpen = false;
    if (maxRow > 0)
    {
        currentRow++;
        if (m_fileName.isEmpty())
        {
            needOpen = true;
        }
        else if (currentRow >= maxRow)
        {
            needOpen = true;
        }
    }
    else if (maxSize > 0)
    {
        // 1MB=1024*1024 经过大量测试 QFile().size() 方法速度非常快
        //首次需要重新打开文件以及超过大小需要重新打开文件
        if (m_fileName.isEmpty())
        {
            needOpen = true;
        }
        else if (file->size() > (maxSize * 1024))
        {
            needOpen = true;
        }
    }
    else
    {
        //日期改变了才会触发
        QString fileName = QString("%1/%2_log_%3.txt").arg(path).arg(name).arg(QDATE);
        openFile(fileName);
    }

    if ((maxRow > 0 || maxSize > 0) && needOpen)
    {
        currentRow       = 0;
        QString fileName = QString("%1/%2_log_%3.txt").arg(path).arg(name).arg(QDATETIMS);
        openFile(fileName);
    }
    //用文本流的输出速度更快
    QTextStream stream(file);
    stream << content << "\n";
}

void SaveLog::setMaxRow(int maxRow)
{
    //这里可以限定最大最小值
    if (maxRow >= 0)
    {
        this->maxRow = maxRow;
        this->clear();
    }
}

void SaveLog::setMaxSize(int maxSize)
{
    //这里可以限定最大最小值
    if (maxSize >= 0)
    {
        this->maxSize = maxSize;
        this->clear();
    }
}

void SaveLog::setPath(const QString& path)
{
    this->path = path;
}

void SaveLog::setName(const QString& name)
{
    this->name = name;
}

void SaveLog::setMsgType(const MsgType& msgType)
{
    this->msgType = msgType;
}

void SaveLog::onTimerClear()
{
    QDir        dir(this->path);
    QStringList fileListFilter;
    fileListFilter.append("*.txt");
    QFileInfoList fileList = dir.entryInfoList(fileListFilter, QDir::Files | QDir::Readable | QDir::Writable | QDir::Hidden | QDir::NoDotAndDotDot,
                                               QDir::Time | QDir::Reversed);   //按时间排序
    //遍历日志文件，判断时间是否过期
    QDateTime today = QDateTime::currentDateTime();
    foreach (auto info, fileList)
    {
        QDateTime date      = info.lastModified();
        uint      filedate  = date.toTime_t();
        uint      todayData = today.toTime_t();
        int       ruler     = todayData - filedate;
        ruler /= (60 * 60 * 24);
        if (ruler > 7)
        {
            QFile::remove(info.filePath());
        }
    }
}
