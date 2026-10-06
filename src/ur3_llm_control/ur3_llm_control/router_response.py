"""Decode non-stream JSON or OpenAI SSE; retain only safe structural diagnostics."""
import gzip
import io
import json


class ResponseError(ValueError):
    pass


def decode_response(raw, content_type='application/json', content_encoding='', diagnostic=None):
    info = diagnostic if diagnostic is not None else {}
    info.update(byte_count=len(raw), content_type=content_type, content_encoding=content_encoding)
    if content_encoding.lower() == 'gzip':
        with gzip.GzipFile(fileobj=io.BytesIO(raw)) as stream:
            raw = stream.read(131073)
        if len(raw) > 131072:
            raise ResponseError('Expanded response exceeds 128 KiB')
    elif content_encoding.lower() not in ('', 'identity'):
        raise ResponseError('Unsupported response content encoding')
    try:
        text = raw.decode('utf-8-sig')
        is_sse = 'text/event-stream' in content_type or text.lstrip().startswith(('data:', 'event:', ':'))
        if not is_sse:
            info['wire_format'] = 'json'
            body = json.loads(text)
            info['root_type'] = type(body).__name__
            info['root_keys'] = [k for k in body if k in ('choices','error','candidates','object','id','usage','model')] if isinstance(body,dict) else []
            if not isinstance(body,dict) or not isinstance(body.get('choices'),list) or len(body['choices']) != 1:
                raise ResponseError('JSON response must contain exactly one choices entry')
            choice = body['choices'][0]
            if choice.get('finish_reason') in ('length','content_filter'):
                raise ResponseError('Model output was truncated or filtered')
            content = choice['message']['content']
            info['content_path'] = 'choices[0].message.content'
        else:
            info.update(wire_format='sse', content_path='choices[0].delta.content', frame_count=0)
            parts, terminal = [], False
            # A complete read is bounded by urllib's socket timeout and size cap.
            pending = []
            frames = []
            for line in text.splitlines()+['']:
                if not line:
                    if pending: frames.append('\n'.join(pending)); pending=[]
                elif line.startswith('data:'):
                    pending.append(line[5:].lstrip())
            for data in frames:
                if data == '[DONE]':
                    terminal = True
                    break
                frame = json.loads(data)
                info['frame_count'] += 1
                if not isinstance(frame,dict) or 'error' in frame:
                    raise ResponseError('Router returned an SSE error frame')
                choices = frame.get('choices')
                if choices == []:  # optional usage-only frame
                    continue
                if not isinstance(choices,list) or len(choices) != 1:
                    raise ResponseError('SSE frame must contain exactly one choices entry')
                choice = choices[0]
                finish = choice.get('finish_reason')
                if finish in ('length','content_filter'):
                    raise ResponseError('Model output was truncated or filtered')
                if finish is not None: terminal = True
                delta = choice.get('delta', choice.get('message',{}))
                fragment = delta.get('content')
                if fragment is not None:
                    if not isinstance(fragment,str):
                        raise ResponseError('SSE text content is not a string')
                    parts.append(fragment)
            if not terminal:
                raise ResponseError('SSE response ended without terminal event')
            content = ''.join(parts)
        if not isinstance(content,str) or not content.strip():
            raise ResponseError('Response has no nonempty text content')
        return content
    except (ValueError, KeyError, TypeError, AttributeError, UnicodeError, OSError, EOFError) as exc:
        if isinstance(exc,ResponseError): raise
        raise ResponseError('Malformed response framing or choices/message/content structure') from None
