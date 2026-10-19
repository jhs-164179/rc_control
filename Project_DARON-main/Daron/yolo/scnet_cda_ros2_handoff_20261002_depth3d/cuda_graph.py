"""Optional fixed-shape CUDA replay for the PyTorch inference backend."""
import warnings
import torch


class GraphForward:
    """Capture one input shape. Other shapes safely use the original forward.

    One detector worker owns this callable, as in the ROS latest-pair worker.
    Postprocessing, NMS, thresholds and output coordinates stay in Ultralytics.
    """
    def __init__(self, forward):
        self.forward=forward
        self.graph=None
        self.disabled=False
        self.key=None
        self.capture_error=None

    def __call__(self, im, *args, **kwargs):
        if self.disabled or not im.is_cuda or kwargs.get('augment') or kwargs.get('visualize') or kwargs.get('embed'):
            return self.forward(im,*args,**kwargs)
        key=(tuple(im.shape),im.dtype,im.device)
        if self.graph is not None and key!=self.key:
            # A dynamic Detect head may replace cached anchor tensors. Do not
            # replay a graph referencing old caches after a shape change.
            self.disabled=True
            return self.forward(im,*args,**kwargs)
        if self.graph is None:
            try:
                self.static=torch.empty_like(im);self.static.copy_(im)
                stream=torch.cuda.Stream(device=im.device)
                stream.wait_stream(torch.cuda.current_stream(im.device))
                with torch.cuda.stream(stream):
                    for _ in range(3): self.forward(self.static,*args,**kwargs)
                torch.cuda.current_stream(im.device).wait_stream(stream)
                graph=torch.cuda.CUDAGraph()
                with torch.cuda.graph(graph,stream=stream):
                    self.output=self.forward(self.static,*args,**kwargs)
                self.graph=graph;self.key=key
            except (RuntimeError,NotImplementedError) as exc:
                self.disabled=True;self.capture_error=str(exc)
                warnings.warn(f'CUDA graph unavailable; ordinary inference retained: {exc}',RuntimeWarning)
                return self.forward(im,*args,**kwargs)
        self.static.copy_(im)
        self.graph.replay()
        return self.output
