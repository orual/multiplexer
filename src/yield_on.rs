use core::future::Future;
use core::pin::Pin;
use core::ptr;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

static VTABLE: RawWakerVTable = RawWakerVTable::new(|_| RawWaker::new(ptr::null(), &VTABLE), |_| {}, |_| {}, |_| {});

/// Run a future to completion using a busy loop.
///
/// This calls `.poll()` on the future in a busy loop, which blocks
/// the current thread at 100% cpu usage until the future is done. The
/// future's `Waker` mechanism is not used.
///
/// You can use this to run multiple futures concurrently with [`join`][crate::join].
///
/// It's suitable for systems with no or limited concurrency and without
/// strict requirements around power consumption. For more complex use
/// cases, prefer using a "real" executor like `embassy-executor`, which
/// supports multiple tasks, and putting the core to sleep when no task
/// needs to do work.
pub fn block_on<F: Future>(mut fut: F) -> F::Output {
    // safety: we don't move the future after this line.
    let mut fut = unsafe { Pin::new_unchecked(&mut fut) };

    let raw_waker = RawWaker::new(ptr::null(), &VTABLE);
    let waker = unsafe { Waker::from_raw(raw_waker) };
    let mut cx = Context::from_waker(&waker);
    loop {
        if let Poll::Ready(res) = fut.as_mut().poll(&mut cx) {
            return res;
        }
    }
}

pub fn yield_now() -> impl Future<Output = ()> {
    YieldNowFuture { yielded: false }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct YieldNowFuture {
    yielded: bool,
}

impl Future for YieldNowFuture {
    type Output = ();
    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        if self.yielded {
            Poll::Ready(())
        } else {
            self.yielded = true;
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    }
}