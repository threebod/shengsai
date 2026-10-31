#include <QtTest>
#include <QDir>
#include <QFontDatabase>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQmlProperty>
#include <QQuickItem>
#include <QQuickStyle>
#include <QQuickWindow>
#include <QtMath>
#include "../upper_computer/Controller.h"

class LayoutTest : public QObject {
    Q_OBJECT
private slots:
    void safeViewport_data() {
        QTest::addColumn<QSize>("size");
        QTest::addColumn<QMargins>("insets");
        QTest::newRow("small-phone") << QSize(320, 640) << QMargins(0, 32, 0, 24);
        QTest::newRow("gesture-phone") << QSize(360, 800) << QMargins(0, 40, 0, 24);
        QTest::newRow("navigation-buttons") << QSize(430, 932) << QMargins(0, 32, 0, 48);
        QTest::newRow("landscape-cutout") << QSize(740, 360) << QMargins(44, 0, 0, 24);
        QTest::newRow("desktop") << QSize(1280, 860) << QMargins();
    }

    void safeViewport() {
        QFETCH(QSize, size);
        QFETCH(QMargins, insets);
        Controller controller;
        QQmlApplicationEngine engine;
        QStringList warnings;
        connect(&engine, &QQmlEngine::warnings, this, [&warnings](const QList<QQmlError> &errors) {
            for (const auto &error : errors) warnings.append(error.toString());
        });
        engine.rootContext()->setContextProperty("controller", &controller);
        engine.load(QUrl::fromLocalFile(QStringLiteral(SHENGSAI_SOURCE_DIR "/upper_computer/Main.qml")));
        QVERIFY2(!engine.rootObjects().isEmpty(), qPrintable(warnings.join('\n')));
        auto *window = qobject_cast<QQuickWindow *>(engine.rootObjects().first());
        QVERIFY(window);
        window->resize(size);
        auto setInset = [&](const char *side, int value) {
            return QQmlProperty(window, QStringLiteral("SafeArea.additionalMargins.") + side,
                                qmlContext(window)).write(value);
        };
        QVERIFY(setInset("top", insets.top()));
        QVERIFY(setInset("bottom", insets.bottom()));
        QVERIFY(setInset("left", insets.left()));
        QVERIFY(setInset("right", insets.right()));
        auto *top = window->findChild<QQuickItem *>("topBarContent");
        auto *bottom = window->findChild<QQuickItem *>("bottomBarContent");
        auto *scroll = window->findChild<QQuickItem *>("pageScroll");
        auto *navigation = window->findChild<QQuickItem *>("navigationBar");
        QVERIFY(top && bottom && scroll && navigation);
        auto sceneRect = [](QQuickItem *item) {
            return QRectF(item->mapToScene(QPointF()), item->size());
        };
        auto insideSafeArea = [&](QQuickItem *item) {
            const auto rect = sceneRect(item);
            return rect.left() >= insets.left() - 1 && rect.top() >= insets.top() - 1
                && rect.right() <= size.width() - insets.right() + 1
                && rect.bottom() <= size.height() - insets.bottom() + 1;
        };
        for (int page = 0; page < 5; ++page) {
            window->setProperty("page", page);
            QTest::qWait(100);
            if (page == 0) {
                auto *map = window->findChild<QQuickItem *>("obstacleMap");
                QVERIFY(map && map->isVisible());
                auto findRegion = [&](int number) -> QQuickItem * {
                    const QString name = QString("obstacleRegion%1").arg(number);
                    for (auto *item : map->childItems()) if (item->objectName() == name) return item;
                    return nullptr;
                };
                QCOMPARE(window->property("obstacleCount").toInt(),0);
                controller.connected_ = controller.confirmed_ = true;
                emit controller.changed();
                auto checkButtons = [&](bool enabled) {
                    for (const char *name : {"missionStartButton","missionPrepareButton","routeStartButton"}) {
                        auto *button = window->findChild<QQuickItem *>(name);
                        QVERIFY(button);
                        QCOMPARE(button->isEnabled(),enabled);
                    }
                };
                checkButtons(true);
                auto *region1 = findRegion(1);
                QVERIFY(region1 && QMetaObject::invokeMethod(region1,"clicked"));
                QCOMPARE(window->property("obstacleCount").toInt(),1);
                checkButtons(true);
                controller.busy_ = true; emit controller.changed();
                checkButtons(false);
                controller.busy_ = false; emit controller.changed();
                QVERIFY(QMetaObject::invokeMethod(region1,"clicked"));
                checkButtons(true);
                for (int n : {5,12,15}) {
                    auto *region = findRegion(n);
                    QVERIFY(region && region->height() >= 44);
                    QVERIFY(QMetaObject::invokeMethod(region,"clicked"));
                }
                QCOMPARE(window->property("obstacleMask").toInt(),0x0810);
                QCOMPARE(window->property("obstacleCount").toInt(),2);
                checkButtons(true);
                auto *region5 = findRegion(5);
                auto *region15 = findRegion(15);
                QVERIFY(QMetaObject::invokeMethod(region5,"clicked"));
                QCOMPARE(window->property("obstacleCount").toInt(),1);
                checkButtons(true);
                QVERIFY(QMetaObject::invokeMethod(region15,"clicked"));
                QCOMPARE(window->property("obstacleMask").toInt(),0x4800);
                controller.connected_ = controller.confirmed_ = false;
                emit controller.changed();
                for (const char *name : {"missionStartButton","missionPrepareButton","routeStartButton"}) {
                    auto *button = window->findChild<QQuickItem *>(name);
                    QVERIFY(button && !button->isEnabled());
                }
                window->setProperty("obstacleMask",0x0810);
                const auto captureDir = qEnvironmentVariable("SS_LAYOUT_CAPTURE_DIR");
                if (!captureDir.isEmpty()) {
                    QDir().mkpath(captureDir);
                    auto *flickable = scroll->property("contentItem").value<QQuickItem *>();
                    QVERIFY(flickable);
                    const auto topY = sceneRect(map).top()-sceneRect(scroll).top();
                    flickable->setProperty("contentY",qMax(0.0,topY));
                    QTest::qWait(50);
                    QVERIFY(window->grabWindow().save(captureDir + "/" + QTest::currentDataTag() + "-obstacles.png"));
                    flickable->setProperty("contentY",0);
                }
            }
            if (page == 1) {
                auto *pickupRead = window->findChild<QQuickItem *>("pickupReadButton");
                auto *pickupSave = window->findChild<QQuickItem *>("pickupSaveButton");
                QVERIFY(pickupRead && pickupSave && !pickupRead->isEnabled() && !pickupSave->isEnabled());
                QVERIFY(pickupRead->height() >= 44 && pickupSave->height() >= 44);
                controller.connected_ = true;emit controller.changed();
                QVERIFY(pickupRead->isEnabled() && !pickupSave->isEnabled());
                controller.pickupRead_ = true;emit controller.changed();
                QVERIFY(pickupSave->isEnabled());
                controller.busy_ = true;emit controller.changed();
                QVERIFY(!pickupRead->isEnabled() && !pickupSave->isEnabled());
                controller.busy_ = controller.connected_ = controller.pickupRead_ = false;emit controller.changed();
                auto *heightsCard = window->findChild<QQuickItem *>("placementHeightsCard");
                auto *readButton = window->findChild<QQuickItem *>("heightsReadButton");
                auto *saveButton = window->findChild<QQuickItem *>("heightsSaveButton");
                QVERIFY(heightsCard && heightsCard->isVisible());
                QVERIFY(readButton && saveButton && !readButton->isEnabled() && !saveButton->isEnabled());
                QVERIFY(readButton->height() >= 44 && saveButton->height() >= 44);
                auto *flickable = scroll->property("contentItem").value<QQuickItem *>();
                QVERIFY(flickable && flickable->property("contentY").isValid());
                flickable->setProperty("contentY", qMax(0.0, flickable->property("contentHeight").toReal() - flickable->height()));
                QTest::qWait(50);
            }
            if (page == 3) {
                auto *testButton = window->findChild<QQuickItem *>("axis6TestButton");
                QVERIFY(testButton && testButton->isVisible());
                QVERIFY(!testButton->isEnabled());
                QVERIFY(testButton->height() >= 44);
                auto *lateralCard = window->findChild<QQuickItem *>("lateralTestCard");
                auto *leftTest = window->findChild<QQuickItem *>("lateralLeftButton");
                auto *rightTest = window->findChild<QQuickItem *>("lateralRightButton");
                QVERIFY(lateralCard && lateralCard->isVisible() && leftTest && rightTest);
                QVERIFY(!leftTest->isEnabled() && !rightTest->isEnabled());
                QVERIFY(leftTest->height() >= 44 && rightTest->height() >= 44);
                auto *flickable = scroll->property("contentItem").value<QQuickItem *>();
                QVERIFY(flickable);
                flickable->setProperty("contentY", qMax(0.0, flickable->property("contentHeight").toReal() - flickable->height()));
                QTest::qWait(50);
            }
            if (page == 4) {
                auto *radar = window->findChild<QQuickItem *>("radarPage");
                auto *start = window->findChild<QQuickItem *>("lidarStartButton");
                auto *stop = window->findChild<QQuickItem *>("lidarStopButton");
                auto *plot = window->findChild<QQuickItem *>("lidarPlot");
                QVERIFY(radar && radar->isVisible() && start && stop && plot);
                QVERIFY(!start->isEnabled() && !stop->isEnabled());
                controller.connected_ = true;emit controller.changed();
                QVERIFY(start->isEnabled() && !stop->isEnabled());
                controller.lidarRunning_ = true;emit controller.lidarChanged();
                QVERIFY(!start->isEnabled() && stop->isEnabled());
                controller.lidarPending_ = true;emit controller.lidarChanged();
                QVERIFY(!start->isEnabled() && !stop->isEnabled());
                controller.lidarPending_ = controller.lidarRunning_ = controller.connected_ = false;
                emit controller.changed();emit controller.lidarChanged();
                QVERIFY(start->height() >= 44 && stop->height() >= 44);
                QVERIFY(plot->width() > 0 && plot->height() >= 200);
                if (!qEnvironmentVariable("SS_LAYOUT_CAPTURE_DIR").isEmpty()) {
                    controller.lidarPoints_.clear();
                    for (int i = 0; i < 100; ++i) {
                        double x = -1600 + i * 32, y = 1700;
                        controller.lidarPoints_.append(QVariantMap{{"x",x},{"y",y},
                            {"distance",qSqrt(x*x+y*y)},{"angle",qRadiansToDegrees(qAtan2(x,y))},
                            {"intensity",20+i*2},{"flag",0}});
                    }
                    for (int i = 0; i < 35; ++i) {
                        double x = 850, y = -800 + i * 18;
                        controller.lidarPoints_.append(QVariantMap{{"x",x},{"y",y},
                            {"distance",qSqrt(x*x+y*y)},{"angle",qRadiansToDegrees(qAtan2(x,y))},
                            {"intensity",12},{"flag",i%7==0?2:0}});
                    }
                    controller.lidarRunning_ = true;controller.lidarLastScan_ = controller.clock_.elapsed();
                    controller.lidarPeriod_ = 167;
                    controller.lidarStatus_ = QStringLiteral("布局测试 · 模拟点云，未连接实物雷达");
                    emit controller.lidarPointsChanged();emit controller.lidarChanged();QTest::qWait(50);
                }
            }
            QVERIFY(insideSafeArea(top));
            QVERIFY(insideSafeArea(bottom));
            QVERIFY(insideSafeArea(scroll));
            QVERIFY(sceneRect(scroll).top() >= sceneRect(top).bottom());
            QVERIFY(sceneRect(scroll).bottom() <= sceneRect(bottom).top());
            QCOMPARE(navigation->isVisible(), window->property("mobile").toBool());
            // Scroll vertically rather than extending content outside a narrow viewport.
            QVERIFY(scroll->property("contentWidth").toReal() <= scroll->width() + 1);
            for (auto *item : window->findChildren<QQuickItem *>()) {
                if (!item->isVisible() || !item->property("down").isValid()) continue;
                const auto rect = sceneRect(item);
                QVERIFY2(rect.left() >= insets.left() - 1
                         && rect.right() <= size.width() - insets.right() + 1,
                         qPrintable(item->property("text").toString()));
            }
            const auto captureDir = qEnvironmentVariable("SS_LAYOUT_CAPTURE_DIR");
            if (!captureDir.isEmpty()) {
                QDir().mkpath(captureDir);
                QVERIFY(window->grabWindow().save(captureDir + "/" + QTest::currentDataTag()
                                                  + QString("-%1.png").arg(page + 1)));
            }
            if (page == 4) {
                if (!captureDir.isEmpty() && size.width() < 760) {
                    auto *plot = window->findChild<QQuickItem *>("lidarPlot");
                    auto *flickable = scroll->property("contentItem").value<QQuickItem *>();
                    const auto y = flickable->property("contentY").toReal()
                        + sceneRect(plot).top() - sceneRect(scroll).top() - 12;
                    flickable->setProperty("contentY",qMax(0.0,y));QTest::qWait(50);
                    QVERIFY(window->grabWindow().save(captureDir + "/" + QTest::currentDataTag() + "-radar-plot.png"));
                    flickable->setProperty("contentY",0);
                }
                controller.resetLidar();
            }
        }
        for (const auto *name : {"connectionDialog", "logDialog", "safetyDialog"}) {
            auto *dialog = window->findChild<QObject *>(name);
            QVERIFY(dialog);
            QVERIFY(QMetaObject::invokeMethod(dialog, "open"));
            QTest::qWait(100);
            const qreal x = dialog->property("x").toReal(), y = dialog->property("y").toReal();
            QVERIFY(x >= insets.left());
            QVERIFY(y >= insets.top());
            QVERIFY(x + dialog->property("width").toReal() <= size.width() - insets.right());
            QVERIFY(y + dialog->property("height").toReal() <= size.height() - insets.bottom());
            auto *content = dialog->property("contentItem").value<QQuickItem *>();
            QVERIFY(content);
            if (QString::fromLatin1(name) == "logDialog") {
                auto *readLog = content->findChild<QQuickItem *>("taskLogReadButton");
                QVERIFY(readLog && readLog->isVisible() && !readLog->isEnabled());
                QVERIFY(readLog->height() >= 44);
            }
            for (auto *item : content->findChildren<QQuickItem *>()) {
                if (!item->isVisible() || !item->property("down").isValid()) continue;
                const auto rect = sceneRect(item);
                QVERIFY2(rect.left() >= x - 1 && rect.right() <= x + dialog->property("width").toReal() + 1,
                         qPrintable(item->property("text").toString()));
            }
            const auto captureDir = qEnvironmentVariable("SS_LAYOUT_CAPTURE_DIR");
            if (!captureDir.isEmpty()) {
                QVERIFY(window->grabWindow().save(captureDir + "/" + QTest::currentDataTag()
                                                  + "-" + name + ".png"));
            }
            QVERIFY(QMetaObject::invokeMethod(dialog, "close"));
        }
        // System insets can change without restarting (gesture navigation / rotation).
        QVERIFY(setInset("top", 0));
        QVERIFY(setInset("bottom", 0));
        QTest::qWait(100);
        QVERIFY(sceneRect(top).top() < 32);
        QVERIFY(sceneRect(bottom).bottom() >= size.height() - 1);
        QVERIFY2(warnings.isEmpty(), qPrintable(warnings.join('\n')));
    }
};

int main(int argc, char **argv) {
    QGuiApplication app(argc, argv);
#ifdef Q_OS_WIN
    // The offscreen platform does not discover Windows system fonts itself.
    QFontDatabase::addApplicationFont(qEnvironmentVariable("WINDIR") + "/Fonts/msyh.ttc");
#endif
    QQuickStyle::setStyle("Basic");
    LayoutTest test;
    return QTest::qExec(&test, argc, argv);
}

#include "test_layout.moc"
