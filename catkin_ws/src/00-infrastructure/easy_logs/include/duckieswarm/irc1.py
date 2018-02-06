#! /usr/bin/env python
#
# Example program using irc.client.
#
# This program is free without restrictions; do anything you like with
# it.
#
# Joel Rosdahl <joel@rosdahl.net>

import json
import math
from multiprocessing import Process, Queue
import os
import random
import shutil
import socket
import sys
from tempfile import gettempdir, mkdtemp
import time
import time

import duckietown_utils as dtu
from duckietown_utils.locate_files_impl import locate_files
from duckietown_utils.system_cmd_imp import system_cmd_result
import irc.client, irc.bot  #@UnresolvedImport


def pinner(queue):
    ## Read from the queue
    while True:
        msg = queue.get()  # Read from the queue and do nothing
        if msg == 'exit': break
#        print('received %s' % msg)
        cmd = ['ipfs', 'pin', 'add', '-r', msg]
        print(" ".join(cmd))
        res = system_cmd_result(cwd='.', cmd=cmd, raise_on_error=True)
        print res.stdout


class TestBot(irc.bot.SingleServerIRCBot):

    def __init__(self, queue, channel, nickname, server, port=6667):
        irc.bot.SingleServerIRCBot.__init__(self, [(server, port)], nickname, nickname)
        self.target = channel
        self.queue = queue

        self.seen = {}
        self.known = set()
        self.tmpdir = mkdtemp(prefix='swarm')

        self.admitted = False
#        os.makedirs(self.tmpdir)

    def on_pubmsg(self, c, e):
#        print('pub', c, e.__dict__)
        self._handle(c, e)

    def _handle(self, c, e):
        arguments = e.arguments[0]
        try:
            j = json.loads(arguments)
        except ValueError:
            return
            pass

        if j['mtype'] == 'advertise':
            ipfs = j['details']['ipfs']
            ipfs = str(ipfs)
            if not ipfs in list(self.seen.values()):
                self.known.add(ipfs)
                self.queue.put(ipfs)
                print('Somebody told me about %s' % ipfs)

    def on_privmsg(self, c, e):
#        print('priv', c, e.__dict__)
        self._handle(c, e)

    def on_welcome(self, connection, event):
        print('welcome: %s' % " ".join(event.arguments))
        connection.join(self.target)
#        self.connection.privmsg(self.target, "hello to everybody")
        self.admitted = True

    def on_join(self, connection, event):
#        print 'on_join', event.__dict__

        source = event.source

        for filename, hashed in self.seen.items():
            self.send_message(['all'], 'advertise', {'ipfs': hashed},
                              target=source)

        users = self.channels[self.target].users()

        print('current users: %s' % users)

    def on_disconnect(self, connection, event):
        print('on_disconnect..')
        connection.join(self.target)

    def send_message(self, to, mtype, details, target=None):
        a = {'version': 1, 'to': to, 'mtype': mtype, 'details': details}
        json_string = json.dumps(a)
        if target is None:
            target = self.target
        self.connection.privmsg(target, json_string)

    def look_for_files(self):
        d = 'logs'
        if not os.path.exists(d):
            msg = 'Logs directory does not exist: %s' % d
            raise ValueError(msg)

        filenames = locate_files(d, '*')
        for f in filenames:
            if not f in self.seen:
                self._add_file(f)

    def _add_file(self, filename):
        b0 = os.path.basename(filename)
        b = b0.replace('.', '_')
        d = os.path.join(self.tmpdir, b)
        if not os.path.exists(d):
            os.mkdir(d)
        dest = os.path.join(d, b0)
        shutil.copy(filename, dest)

        cmd = ['ipfs', 'add', '-r', d]
        res = system_cmd_result(cwd='.', cmd=cmd, raise_on_error=True)
        print res.stdout
        s = res.stdout.strip()
        lines = s.split('\n')
        out = lines[-1].split(' ')
        if (len(out) < 3 or out[0] != 'added' or not out[1].startswith('Qm')):
            msg = 'Invalid output for ipds:\n%s' % dtu.indent(res.stdout, ' > ')
            raise Exception(msg)
        hashed = out[1]

        self.seen[filename] = hashed
#        self.known.add(hashed)
        self.send_message(['all'], 'advertise', {'ipfs': hashed, 'filename': b0})


def main():
    queue = Queue()  # reader() reads from queue
                        # writer() writes to queue
    reader_p = Process(target=pinner, args=((queue),))
    reader_p.daemon = True
    reader_p.start()  # Launch reader() as a separate python process

    server = 'frankfurt.co-design.science'
#    server = 'dorothy-3.local'
    nickname = socket.gethostname() + str(random.randint(0, 100))
    channel = '#duckiebots'
    port = 6667
    delta = 20
    c = TestBot(queue, channel, nickname, server, port)
    if True:
        c._connect()
        last_sent = time.time() - delta
        while not c.admitted:
            print('Waiting for welcome')
            time.sleep(1)
            c.reactor.process_once()

        while True:
#            print('one')
            time.sleep(1)

            t = time.time()
            if t > last_sent + delta:
                c.send_message(to=['all'], mtype='hello', details={})

                print('Mine: %s' % sorted(c.seen.values()))
                print('Known: %s' % sorted(c.known))
                last_sent = t
            c.reactor.process_once()
            c.look_for_files()

    else:
        c.start()
    print('done')
    queue.put('exit')
    pinner.join()


if __name__ == "__main__":
    main()
