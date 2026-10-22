#include <QtTest>
#include <QDir>
#include <QFontDatabase>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQmlProperty>
#include <QQuickItem>
#include <QQuickStyle>
#include <QQuickWindow>
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
        for (int page = 0; page < 4; ++page) {
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
                for (int n : {5,12,15}) {
                    auto *region = findRegion(n);
                    QVERIFY(region && region->height() >= 44);
                    QVERIFY(QMetaObject::invokeMethod(region,"clicked"));
                }
                QCOMPARE(window->property("obstacleMask").toInt(),0x0810);
                QCOMPARE(window->property("obstacleCount").toInt(),2);
                auto *region5 = findRegion(5);
                auto *region15 = findRegion(15);
                QVERIFY(QMetaObject::invokeMethod(region5,"clicked"));
                QVERIFY(QMetaObject::invokeMethod(region15,"clicked"));
                QCOMPARE(window->property("obstacleMask").toInt(),0x4800);
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
            if (page == 3) {
                auto *testButton = window->findChild<QQuickItem *>("axis6TestButton");
                QVERIFY(testButton && testButton->isVisible());
                QVERIFY(!testButton->isEnabled());
                QVERIFY(testButton->height() >= 44);
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
