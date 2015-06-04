import oscP5.*;
import netP5.*;

NetAddress remoteServer;
OscP5 osc;

void setup()
{
  size(400,400);
  osc = new OscP5(this,8000);
  remoteServer = new NetAddress("127.0.0.1",8888);
}

void draw()
{
}

void keyPressed(char key)
{
  OscMessage message = new OscMessage(""+key);
  osc.send(message,remoteServer);
}


