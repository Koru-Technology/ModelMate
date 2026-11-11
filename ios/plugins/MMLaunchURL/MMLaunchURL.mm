// Catches a file shared to ModelMate when the share itself launches the app.
//
// iOS hands that file over once, in scene:willConnectToSession:options:, before Godot loads the C# assembly (and
// before it initializes plugins), so the C# receiver (src/App/IosBridge.cs) would miss it. +load runs when the app
// binary loads, before main(); it wraps Godot's scene delegate method to copy the file into tmp/Received (apps may
// not write to Documents/Inbox, which belongs to iOS), which IosBridge.SweepInbox imports on startup. Files shared while the app is running go through IosBridge directly
// (scene:openURLContexts:).

#import <UIKit/UIKit.h>
#import <objc/runtime.h>

@interface MMLaunchURL : NSObject
@end

@implementation MMLaunchURL

+ (void)load {
	Class cls = NSClassFromString(@"GDTApplicationDelegate");
	SEL sel = @selector(scene:willConnectToSession:options:);
	Method m = cls ? class_getInstanceMethod(cls, sel) : NULL;
	if (!m) {
		NSLog(@"ModelMate: GDTApplicationDelegate scene:willConnectToSession:options: not found; files shared at launch will be missed");
		return;
	}
	typedef void (*Original)(id, SEL, UIScene *, UISceneSession *, UISceneConnectionOptions *);
	Original original = (Original)method_getImplementation(m);
	method_setImplementation(m, imp_implementationWithBlock(^(id self, UIScene *scene, UISceneSession *session, UISceneConnectionOptions *options) {
		for (UIOpenURLContext *context in options.URLContexts) [MMLaunchURL take:context.URL];
		original(self, sel, scene, session, options);
	}));
}

+ (void)take:(NSURL *)url {
	if (!url.isFileURL) return;
	NSLog(@"ModelMate: received at launch %@", url.path);
	NSFileManager *fm = NSFileManager.defaultManager;
	NSURL *docs = [fm URLsForDirectory:NSDocumentDirectory inDomains:NSUserDomainMask].firstObject;
	// already copied in by iOS ("Copy to ModelMate"): swept from there
	NSString *dir = url.URLByDeletingLastPathComponent.path.stringByStandardizingPath;
	if ([dir isEqualToString:[docs URLByAppendingPathComponent:@"Inbox"].path.stringByStandardizingPath]) return;
	NSURL *inbox = [NSURL fileURLWithPath:[NSHomeDirectory() stringByAppendingPathComponent:@"tmp/Received"] isDirectory:YES];
	[fm createDirectoryAtURL:inbox withIntermediateDirectories:YES attributes:nil error:nil];
	NSString *name = url.lastPathComponent.stringByDeletingPathExtension, *ext = url.pathExtension;
	NSURL *dest = [inbox URLByAppendingPathComponent:url.lastPathComponent];
	for (int k = 2; [fm fileExistsAtPath:dest.path]; k++)
		dest = [inbox URLByAppendingPathComponent:[NSString stringWithFormat:@"%@ %d.%@", name, k, ext]];
	// files outside the sandbox (another app's, Files/iCloud) need security-scoped access while copying
	BOOL scoped = [url startAccessingSecurityScopedResource];
	NSError *error = nil;
	if (![fm copyItemAtURL:url toURL:dest error:&error])
		NSLog(@"ModelMate: could not copy shared file %@: %@", url.path, error.localizedDescription);
	if (scoped) [url stopAccessingSecurityScopedResource];
}

@end

// Godot plugin entry points, C++ linkage as Godot declares them (they also make the linker keep this object file, and with it +load)
void mm_launch_url_init(void) {}
void mm_launch_url_deinit(void) {}
