#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickWindow>
#include <QQuickStyle>
#include <QTimer>
#include "Controller.h"

int main(int argc,char **argv){
    QGuiApplication app(argc,argv);app.setApplicationName("省赛控制台");app.setOrganizationName("Shengsai");
    QQuickStyle::setStyle("Basic");Controller controller;
    QQmlApplicationEngine engine;engine.rootContext()->setContextProperty("controller",&controller);
    QObject::connect(&engine,&QQmlApplicationEngine::objectCreationFailed,&app,[]{QCoreApplication::exit(1);},Qt::QueuedConnection);
    engine.loadFromModule("Shengsai","Main");
    const auto args=app.arguments();
    if(!engine.rootObjects().isEmpty()){
        auto *window=qobject_cast<QQuickWindow *>(engine.rootObjects().first());
        if(args.contains("--page"))window->setProperty("page",args.value(args.indexOf("--page")+1).toInt());
        if(args.contains("--mobile")){window->setWidth(430);window->setHeight(1000);}
        if(args.contains("--capture")){
            const auto path=args.value(args.indexOf("--capture")+1);
            QTimer::singleShot(1200,&app,[window,path]{QCoreApplication::exit(window->grabWindow().save(path)?0:2);});
        }
    }
    return app.exec();
}
